import os
import unittest
from pathlib import Path

from tdcma_core import TdcmaCpu, load_tdcma_file, parse_tdcma_text


ROOT = os.path.dirname(os.path.dirname(__file__))
PROJECT_ROOT = Path(ROOT).parent


class CoreTests(unittest.TestCase):
    def load_demo(self) -> TdcmaCpu:
        image = load_tdcma_file(os.path.join(ROOT, "examples", "interrupt_demo.txt"))
        cpu = TdcmaCpu()
        cpu.load_image(image)
        return cpu

    def test_parser_accepts_micro_bytes(self) -> None:
        image = parse_tdcma_text("$P 00 60 13 ; comment\n$M 00 00 01 C1\n$M FF 12 34 56\n")
        self.assertEqual(image.main[0], 0x60)
        self.assertEqual(image.main[1], 0x13)
        self.assertEqual(image.micro[0], 0x0001C1)
        self.assertEqual(image.micro[0xFF], 0x123456)

    def test_parser_accepts_multiple_records_per_line(self) -> None:
        image = parse_tdcma_text(
            "$M 01 006D43 ; PC->AR $M 03 107070 ; MEM->IR\n"
            "$M30 001604 ; RD->A\n"
        )
        self.assertEqual(image.micro[0x01], 0x006D43)
        self.assertEqual(image.micro[0x03], 0x107070)
        self.assertEqual(image.micro[0x30], 0x001604)

    def test_load_fills_td_cma_direct_address_entry_when_txt_omits_it(self) -> None:
        image = parse_tdcma_text("$P 00 C0\n$M 1C 10101D\n$M 1D 10608C\n")
        cpu = TdcmaCpu()
        cpu.load_image(image)
        self.assertEqual(cpu.micro[0x3C], 0x006D5C)
        self.assertTrue(any("micro cell 3C missing" in warning for warning in image.warnings))

    def test_reset_preserves_input_switches(self) -> None:
        cpu = self.load_demo()
        cpu.set_input_binary("10101010")
        cpu.step_instruction()
        cpu.reset()
        self.assertEqual(cpu.in_unit, 0xAA)

    def test_program_write_does_not_change_input_switches(self) -> None:
        cpu = TdcmaCpu()
        cpu.set_input_binary("10101010")
        event = cpu._write_io(0x00, 0x55)
        self.assertEqual(cpu.in_unit, 0xAA)
        self.assertIn("ignore write", event)

    def test_logical_alu_preserves_fc_until_arithmetic_changes_it(self) -> None:
        cpu = TdcmaCpu()
        cpu.micro[0] = (0b0010 << 15) | (0b001 << 12) | (0b001 << 9)
        cpu.reg_a = 0xF0
        cpu.reg_b = 0x0F
        cpu.fc = 1
        cpu.step_micro()
        self.assertEqual(cpu.reg_a, 0x00)
        self.assertEqual(cpu.fz, 1)
        self.assertEqual(cpu.fc, 1)

    def test_ri_indexed_addressing_uses_r2(self) -> None:
        image = parse_tdcma_text(
            """
            $P 00 62 A0
            $P 02 60 5A
            $P 04 D8 00
            $P 06 C9 00
            $P 08 50
            $M 00 000001
            $M 01 006D43
            $M 03 107070
            $M 0C 103001
            $M 0D 200601
            $M 11 103001
            $M 28 101029
            $M 29 00282A
            $M 2A 04E22B
            $M 2B 04928C
            $M 35 000035
            $M 36 006D51
            $M 3E 006D68
            """
        )
        cpu = TdcmaCpu()
        cpu.load_image(image)
        for _ in range(8):
            cpu.step_instruction()
            if cpu.halted:
                break
        self.assertEqual(cpu.mem[0xA0], 0x5A)
        self.assertEqual(cpu.regs[1], 0x5A)

    def test_demo_reaches_interrupt_and_outputs_incremented_value(self) -> None:
        cpu = self.load_demo()
        cpu.set_input_binary("00000011")
        for _ in range(30):
            cpu.step_instruction()
            if cpu.ei and cpu.pc == 0x15:
                break
        self.assertEqual(cpu.ei, 1)
        cpu.trigger_irq0()
        for _ in range(20):
            cpu.step_instruction()
            if cpu.out_unit == 0x04:
                break
        self.assertEqual(cpu.out_unit, 0x04)
        self.assertFalse(cpu.pic.intr)

    def test_loop_control_interrupt_commands(self) -> None:
        image = load_tdcma_file(os.path.join(ROOT, "examples", "TD_CMA_interrupt_loop_control_fixed.txt"))
        cpu = TdcmaCpu()
        cpu.load_image(image)
        for index in range(80):
            cpu.step_instruction()
            if cpu.ei and cpu.pc == 0x20 and index > 12:
                break

        def irq(command: int) -> None:
            cpu.set_input_binary(f"{command:08b}")
            cpu.trigger_irq0()
            for step in range(160):
                cpu.step_instruction()
                if not cpu.pic.intr and cpu.ei and cpu.pc < 0x80 and step > 3:
                    break

        irq(0)
        self.assertEqual(cpu.mem[0xF9], 1)
        irq(0)
        self.assertEqual(cpu.mem[0xF9], 0)
        irq(1)
        self.assertEqual(cpu.mem[0xFA], 1)
        irq(2)
        self.assertEqual(cpu.mem[0xFB], 2)
        cpu.mem[0xF8] = 5
        cpu.mem[0xF9] = 1
        irq(3)
        self.assertEqual([cpu.mem[a] for a in range(0xF8, 0xFC)], [0, 0, 0, 1])

        irq(1)
        irq(2)
        seen = []
        for _ in range(160):
            cpu.step_instruction()
            if cpu.pc == 0x17:
                seen.append(cpu.out_unit)
            if len(seen) >= 5:
                break
        self.assertEqual(seen[:5], [8, 6, 4, 2, 0])

    def test_final_project_txt_four_modes_survive_two_irq_flow(self) -> None:
        txt = next(p for p in PROJECT_ROOT.iterdir() if p.suffix.lower() == ".txt" and "MEM" in p.name and "微程序" in p.name)
        image = load_tdcma_file(str(txt))
        cpu = TdcmaCpu()
        cpu.load_image(image)

        for _ in range(120):
            cpu.step_instruction()
            if cpu.ei and cpu.pc == 0x15:
                break

        def settle(limit: int = 320) -> None:
            for _ in range(limit):
                cpu.step_instruction()
                if not cpu.pic.intr and cpu.ei and cpu.pc == 0x15:
                    return

        def command(cmd: int) -> None:
            cpu.trigger_irq0()
            settle()
            cpu.set_input_binary(f"{cmd:08b}")
            cpu.trigger_irq0()
            settle()

        def observe(n: int, limit: int = 180) -> list[int]:
            vals: list[int] = []
            for _ in range(n):
                for __ in range(limit):
                    cpu.step_instruction()
                    if cpu.pc == 0x15:
                        vals.append(cpu.out_unit)
                        break
            return vals

        command(0x05)
        self.assertEqual(cpu.mem[0xE7], 0x00)
        self.assertEqual(cpu.mem[0xE8], 0x05)
        self.assertEqual(cpu.mem[0xF1], 0x05)

        command(0x45)
        self.assertEqual(cpu.mem[0xE7], 0x40)
        self.assertEqual(cpu.mem[0xF1], 0x05)
        self.assertEqual(observe(4)[:4], [5, 10, 15, 4])

        command(0x85)
        self.assertEqual(cpu.mem[0xE7], 0x80)
        self.assertEqual(cpu.mem[0xF1], 0x05)
        self.assertEqual(observe(4, limit=320)[:4], [9, 4, 15, 10])

        command(0xC0)
        self.assertEqual(cpu.mem[0xE7], 0xC0)
        self.assertEqual(cpu.mem[0xE8], 0x00)
        self.assertEqual(observe(1), [0])

    def test_final_project_txt_mode_decode_stays_correct_with_dirty_fc(self) -> None:
        txt = next(p for p in PROJECT_ROOT.iterdir() if p.suffix.lower() == ".txt" and "MEM" in p.name and "微程序" in p.name)
        image = load_tdcma_file(str(txt))
        cpu = TdcmaCpu()
        cpu.load_image(image)

        for _ in range(120):
            cpu.step_instruction()
            if cpu.ei and cpu.pc == 0x15:
                break

        def settle(limit: int = 320) -> None:
            for _ in range(limit):
                cpu.step_instruction()
                if not cpu.pic.intr and cpu.ei and cpu.pc == 0x15:
                    return

        def command(cmd: int, dirty_fc: bool = False) -> None:
            cpu.trigger_irq0()
            settle()
            cpu.set_input_binary(f"{cmd:08b}")
            if dirty_fc:
                cpu.fc = 1
            cpu.trigger_irq0()
            settle()

        command(0x05, dirty_fc=True)
        self.assertEqual((cpu.mem[0xE7], cpu.mem[0xE8], cpu.mem[0xF1], cpu.mem[0xF0]), (0x00, 0x05, 0x05, 0x00))

        command(0x45, dirty_fc=True)
        self.assertEqual((cpu.mem[0xE7], cpu.mem[0xF1], cpu.mem[0xF0]), (0x40, 0x05, 0x00))

        command(0x85, dirty_fc=True)
        self.assertEqual((cpu.mem[0xE7], cpu.mem[0xF1], cpu.mem[0xF0]), (0x80, 0x05, 0x00))

        command(0xC0, dirty_fc=True)
        self.assertEqual((cpu.mem[0xE7], cpu.mem[0xE8], cpu.mem[0xF0]), (0xC0, 0x00, 0x00))


if __name__ == "__main__":
    unittest.main()
