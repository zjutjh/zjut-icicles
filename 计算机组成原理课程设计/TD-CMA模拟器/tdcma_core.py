from __future__ import annotations

from dataclasses import dataclass, field
import re
from typing import Dict, Iterable, List, Optional, Tuple


BYTE_MASK = 0xFF
MICRO_ADDR_MASK = 0xFF
MICRO_NEXT_MASK = 0x3F


class ParseError(ValueError):
    pass


@dataclass
class MemoryImage:
    main: Dict[int, int] = field(default_factory=dict)
    micro: Dict[int, int] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class MicroInstruction:
    word: int

    @property
    def m23(self) -> int:
        return (self.word >> 23) & 1

    @property
    def inta(self) -> int:
        return (self.word >> 22) & 1

    @property
    def wr(self) -> int:
        return (self.word >> 21) & 1

    @property
    def rd(self) -> int:
        return (self.word >> 20) & 1

    @property
    def iom(self) -> int:
        return (self.word >> 19) & 1

    @property
    def s(self) -> int:
        return (self.word >> 15) & 0xF

    @property
    def a(self) -> int:
        return (self.word >> 12) & 0x7

    @property
    def b(self) -> int:
        return (self.word >> 9) & 0x7

    @property
    def c(self) -> int:
        return (self.word >> 6) & 0x7

    @property
    def ma(self) -> int:
        return self.word & MICRO_NEXT_MASK

    def control_summary(self) -> str:
        pieces: List[str] = []
        if self.inta:
            pieces.append("INTA#")
        if self.wr:
            pieces.append("WR")
        if self.rd:
            pieces.append("RD")
        if self.iom:
            pieces.append("IOM")
        pieces.append(f"S={self.s:04b}")
        pieces.append(f"A={A_FIELD_NAMES[self.a]}")
        pieces.append(f"B={B_FIELD_NAMES[self.b]}")
        pieces.append(f"C={C_FIELD_NAMES[self.c]}")
        pieces.append(f"MA={self.ma:02X}")
        return " ".join(pieces)


@dataclass
class StepEvent:
    micro_addr: int
    micro_word: int
    summary: str
    bus_source: Optional[str] = None
    bus_value: Optional[int] = None
    changes: List[str] = field(default_factory=list)
    io_event: Optional[str] = None
    instruction_event: Optional[str] = None
    warning: Optional[str] = None

    def format(self) -> str:
        head = f"u{self.micro_addr:02X} {self.micro_word:06X}  {self.summary}"
        details: List[str] = []
        if self.bus_source is not None and self.bus_value is not None:
            details.append(f"BUS[{self.bus_source}]={self.bus_value:02X}")
        if self.io_event:
            details.append(self.io_event)
        if self.instruction_event:
            details.append(self.instruction_event)
        if self.changes:
            details.append(", ".join(self.changes))
        if self.warning:
            details.append(f"WARN: {self.warning}")
        if details:
            return head + "  |  " + "  |  ".join(details)
        return head


A_FIELD_NAMES = {
    0b000: "NOP",
    0b001: "LDA",
    0b010: "LDB",
    0b011: "LDRi",
    0b100: "LDSP",
    0b101: "LOAD",
    0b110: "LDAR",
    0b111: "LDIR",
}

B_FIELD_NAMES = {
    0b000: "NOP",
    0b001: "ALU_B",
    0b010: "RS_B",
    0b011: "RD_B",
    0b100: "RI_B",
    0b101: "SP_B",
    0b110: "PC_B",
    0b111: "RESERVED",
}

C_FIELD_NAMES = {
    0b000: "NOP",
    0b001: "P<1>",
    0b010: "P<2>",
    0b011: "P<3>",
    0b100: "P<4>",
    0b101: "LDPC",
    0b110: "STI",
    0b111: "CLI",
}


REG_NAMES = ("R0", "R1", "R2", "R3")
TD_CMA_DIRECT_MODE_ENTRY = 0x3C
TD_CMA_DIRECT_MODE_WORD = 0x006D5C


def parse_tdcma_text(text: str) -> MemoryImage:
    image = MemoryImage()
    for line_no, raw in enumerate(text.splitlines(), 1):
        if _is_comment_only(raw):
            continue
        matches = list(_RECORD_START_RE.finditer(raw))
        if not matches:
            continue
        for index, match in enumerate(matches):
            tag = "$" + match.group(1).upper()
            addr_token = match.group(2)
            body_end = matches[index + 1].start() if index + 1 < len(matches) else len(raw)
            tokens = _record_tokens(raw[match.end() : body_end])
            if not tokens:
                image.warnings.append(f"line {line_no}: ignored incomplete {tag} line")
                continue
            addr = int(addr_token, 16)
            if tag == "$P":
                _parse_program_record(image, line_no, addr, tokens)
            else:
                _parse_micro_record(image, line_no, addr, tokens)
    if not image.main:
        image.warnings.append("no $P main memory records found")
    if not image.micro:
        image.warnings.append("no $M micro memory records found")
    return image


def _parse_program_record(image: MemoryImage, line_no: int, addr: int, tokens: List[str]) -> None:
    if not 0 <= addr <= 0xFF:
        raise ParseError(f"line {line_no}: main memory address out of range")
    offset = 0
    for token in tokens:
        if not _is_hex(token):
            break
        value = int(token, 16)
        if not 0 <= value <= 0xFF:
            raise ParseError(f"line {line_no}: program byte out of range")
        image.main[(addr + offset) & BYTE_MASK] = value
        offset += 1


def _parse_micro_record(image: MemoryImage, line_no: int, addr: int, tokens: List[str]) -> None:
    if not 0 <= addr <= MICRO_ADDR_MASK:
        raise ParseError(f"line {line_no}: micro address out of range")
    data_hex = _collect_micro_hex(tokens)
    if len(data_hex) != 6:
        raise ParseError(
            f"line {line_no}: micro word must be 24-bit hex, got {data_hex!r}"
        )
    image.micro[addr] = int(data_hex, 16) & 0xFFFFFF


def load_tdcma_file(path: str) -> MemoryImage:
    with open(path, "r", encoding="utf-8-sig") as file:
        return parse_tdcma_text(file.read())


_RECORD_START_RE = re.compile(r"\$([PpMm])\s*([0-9A-Fa-f]{1,2})(?=\s)")


def _is_comment_only(line: str) -> bool:
    stripped = line.lstrip()
    return stripped.startswith("//") or stripped.startswith("#")


def _record_tokens(fragment: str) -> List[str]:
    return fragment.replace(";", " ; ").replace("//", " // ").split()


def _program_uses_direct_memory_mode(main: Dict[int, int]) -> bool:
    for value in main.values():
        op = (value >> 4) & 0x0F
        mode = (value >> 2) & 0x03
        if 0x0C <= op <= 0x0F and mode == 0:
            return True
    return False


def _is_hex(token: str) -> bool:
    return bool(re.fullmatch(r"[0-9A-Fa-f]+", token))


def _collect_micro_hex(tokens: Iterable[str]) -> str:
    pieces: List[str] = []
    for token in tokens:
        if not _is_hex(token):
            break
        pieces.append(token)
        if sum(len(part) for part in pieces) >= 6:
            break
    return "".join(pieces).upper()


class Pic8259:
    """Small 8259 model tailored to the TD-CMA interrupt experiment."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.icw1 = 0
        self.icw2 = 0x30
        self.icw3 = 0
        self.icw4 = 0
        self.ocw1 = 0xFF
        self.irr = 0
        self.isr = 0
        self.initialized = False
        self._init_step = 0
        self._needs_icw3 = False
        self._needs_icw4 = False
        self._ack_phase = 0
        self._ack_level: Optional[int] = None

    @property
    def imr(self) -> int:
        return self.ocw1 & BYTE_MASK

    @property
    def intr(self) -> bool:
        return self._active_level() is not None

    @property
    def vector_base(self) -> int:
        return self.icw2 & 0xF8

    def request(self, level: int) -> None:
        if not 0 <= level <= 7:
            raise ValueError("8259 IRQ level must be 0..7")
        self.irr |= 1 << level

    def write(self, port: int, value: int) -> str:
        port &= BYTE_MASK
        value &= BYTE_MASK
        a0 = port & 1
        if a0 == 0 and (value & 0x10):
            self.icw1 = value
            self.initialized = False
            self._needs_icw3 = not bool(value & 0x02)
            self._needs_icw4 = bool(value & 0x01)
            self._init_step = 2
            return f"8259 ICW1={value:02X}"
        if self._init_step == 2 and a0 == 1:
            self.icw2 = value
            if self._needs_icw3:
                self._init_step = 3
            elif self._needs_icw4:
                self._init_step = 4
            else:
                self._finish_init()
            return f"8259 ICW2={value:02X}"
        if self._init_step == 3 and a0 == 1:
            self.icw3 = value
            if self._needs_icw4:
                self._init_step = 4
            else:
                self._finish_init()
            return f"8259 ICW3={value:02X}"
        if self._init_step == 4 and a0 == 1:
            self.icw4 = value
            self._finish_init()
            return f"8259 ICW4={value:02X}"
        if a0 == 1:
            self.ocw1 = value
            return f"8259 OCW1/IMR={value:02X}"
        if value & 0x20:
            self.isr = 0
            return f"8259 OCW2 EOI={value:02X}"
        return f"8259 OCW2/3={value:02X}"

    def read(self, port: int) -> Tuple[int, str]:
        a0 = port & 1
        if a0:
            return self.imr, f"8259 IMR->{self.imr:02X}"
        return self.irr & BYTE_MASK, f"8259 IRR->{self.irr & BYTE_MASK:02X}"

    def acknowledge(self) -> Tuple[Optional[int], str]:
        if self._ack_phase == 0:
            level = self._active_level()
            if level is None:
                return None, "INTA# with no pending IRQ"
            self._ack_level = level
            self._ack_phase = 1
            return None, f"8259 first INTA#, IRQ{level} latched"
        level = self._ack_level
        self._ack_phase = 0
        self._ack_level = None
        if level is None:
            return self.vector_base, "8259 second INTA#, no latched IRQ"
        self.irr &= ~(1 << level)
        vector = (self.vector_base + level) & BYTE_MASK
        if self.icw4 & 0x02:
            self.isr &= ~(1 << level)
        else:
            self.isr |= 1 << level
        return vector, f"8259 vector {vector:02X} for IRQ{level}"

    def _finish_init(self) -> None:
        self.initialized = True
        self._init_step = 0

    def _active_level(self) -> Optional[int]:
        pending = self.irr & ~self.imr & BYTE_MASK
        if pending == 0:
            return None
        for level in range(8):
            if pending & (1 << level):
                return level
        return None


class TdcmaCpu:
    def __init__(self) -> None:
        self.loaded_main = [0] * 256
        self.loaded_micro: List[Optional[int]] = [None] * 256
        self.mem = [0] * 256
        self.micro: List[Optional[int]] = [None] * 256
        self.pic = Pic8259()
        self.reset()

    def load_image(self, image: MemoryImage) -> None:
        self.loaded_main = [0] * 256
        self.loaded_micro = [None] * 256
        for addr, value in image.main.items():
            self.loaded_main[addr & BYTE_MASK] = value & BYTE_MASK
        for addr, value in image.micro.items():
            self.loaded_micro[addr & MICRO_ADDR_MASK] = value & 0xFFFFFF
        self._apply_micro_compatibility_defaults(image)
        self.reset()

    def reset(self) -> None:
        previous_input = getattr(self, "in_unit", 0) & BYTE_MASK
        self.mem = list(getattr(self, "loaded_main", [0] * 256))
        self.micro = list(getattr(self, "loaded_micro", [None] * 256))
        self.regs = [0, 0, 0, 0]
        self.reg_a = 0
        self.reg_b = 0
        self.pc = 0
        self.ar = 0
        self.ir = 0
        self.fc = 0
        self.fz = 0
        self.ei = 0
        self.in_unit = previous_input
        self.out_unit = 0
        self.micro_addr = 0
        self.halted = False
        self.micro_steps = 0
        self.completed_instructions = 0
        self.pic.reset()

    @property
    def sp(self) -> int:
        return self.regs[3]

    @sp.setter
    def sp(self, value: int) -> None:
        self.regs[3] = value & BYTE_MASK

    @property
    def intr(self) -> bool:
        return self.pic.intr

    def state(self) -> Dict[str, int]:
        return {
            "PC": self.pc,
            "IR": self.ir,
            "AR": self.ar,
            "A": self.reg_a,
            "B": self.reg_b,
            "R0": self.regs[0],
            "R1": self.regs[1],
            "R2": self.regs[2],
            "R3": self.regs[3],
            "SP": self.sp,
            "FZ": self.fz,
            "FC": self.fc,
            "EI": self.ei,
            "OUT": self.out_unit,
            "uAR": self.micro_addr,
            "INTR": 1 if self.intr else 0,
        }

    def set_input_binary(self, bits: str) -> None:
        bits = bits.strip().replace("_", "")
        if not re.fullmatch(r"[01]{1,8}", bits):
            raise ValueError("IN value must be 1 to 8 binary digits")
        self.in_unit = int(bits, 2) & BYTE_MASK

    def trigger_irq0(self) -> None:
        self.pic.request(0)

    def _apply_micro_compatibility_defaults(self, image: MemoryImage) -> None:
        # Some TD-CMA txt files exported for the physical machine rely on an
        # existing direct-addressing entry in micro cell 3CH. The real
        # controller is not cleared by those files, so mirror that board state
        # when the surrounding textbook microprogram layout is present.
        if self.loaded_micro[TD_CMA_DIRECT_MODE_ENTRY] is not None:
            return
        if not _program_uses_direct_memory_mode(image.main):
            return
        if self.loaded_micro[0x1C] is None or self.loaded_micro[0x1D] is None:
            return
        self.loaded_micro[TD_CMA_DIRECT_MODE_ENTRY] = TD_CMA_DIRECT_MODE_WORD
        warning = "micro cell 3C missing; filled TD-CMA direct-address entry 006D5C"
        if warning not in image.warnings:
            image.warnings.append(warning)

    def step_instruction(self, max_micro_steps: int = 512) -> List[StepEvent]:
        events: List[StepEvent] = []
        if self.halted:
            return events
        saw_ir_fetch = False
        for _ in range(max_micro_steps):
            event = self.step_micro()
            events.append(event)
            if event.instruction_event:
                saw_ir_fetch = True
            if self.halted:
                break
            if saw_ir_fetch and self.micro_addr == 0x01:
                self.completed_instructions += 1
                break
        else:
            self.halted = True
            events.append(
                StepEvent(
                    self.micro_addr,
                    0,
                    "STOP",
                    warning="microprogram did not reach an instruction boundary",
                )
            )
        return events

    def step_micro(self) -> StepEvent:
        if self.halted:
            return StepEvent(self.micro_addr, 0, "HALTED")
        addr = self.micro_addr & MICRO_ADDR_MASK
        word = self.micro[addr]
        before = self.state()
        if word is None:
            self.halted = True
            return StepEvent(addr, 0, "MISSING", warning="empty micro memory cell")

        mi = MicroInstruction(word)
        vector, inta_event = (None, None)
        if mi.inta:
            vector, inta_event = self.pic.acknowledge()

        bus, bus_source, io_event = self._drive_bus(mi, vector)
        alu_flags = self._pending_alu_flags(mi, bus_source)

        if mi.wr:
            if bus is None:
                bus = 0
                bus_source = bus_source or "FLOAT"
            io_event = self._write_target(mi, bus, io_event)

        if bus is not None:
            self._load_a_field(mi, bus)

        if mi.c == 0b101:
            if mi.a == 0b101:
                self.pc = (bus or 0) & BYTE_MASK
            else:
                self.pc = (self.pc + 1) & BYTE_MASK
        elif mi.c == 0b110:
            self.ei = 1
        elif mi.c == 0b111:
            self.ei = 0

        for name, value in alu_flags.items():
            setattr(self, name, value)

        next_addr = self._next_micro_addr(mi)
        self.micro_addr = next_addr
        self.micro_steps += 1

        if addr == 0x35 and mi.ma == 0x35:
            self.halted = True

        instruction_event = None
        if mi.a == 0b111 and bus is not None:
            operand = self.mem[self.pc]
            instr_pc = (self.pc - 1) & BYTE_MASK
            instruction_event = (
                f"IR@{instr_pc:02X}={self.ir:02X} "
                f"{disassemble(self.ir, operand)}"
            )

        if inta_event:
            io_event = f"{io_event}; {inta_event}" if io_event else inta_event

        after = self.state()
        changes = _format_changes(before, after)
        return StepEvent(
            micro_addr=addr,
            micro_word=word,
            summary=mi.control_summary(),
            bus_source=bus_source,
            bus_value=bus,
            changes=changes,
            io_event=io_event,
            instruction_event=instruction_event,
        )

    def _drive_bus(
        self, mi: MicroInstruction, inta_vector: Optional[int]
    ) -> Tuple[Optional[int], Optional[str], Optional[str]]:
        if mi.rd:
            if mi.iom:
                value, event = self._read_io(self.ar)
                return value, f"IO[{self.ar:02X}]", event
            value = self.mem[self.ar]
            return value, f"MEM[{self.ar:02X}]", None
        if mi.b == 0b001:
            value, _ = self._alu(mi)
            return value, "ALU", None
        if mi.b == 0b010:
            index = (self.ir >> 2) & 0x3
            return self.regs[index], f"RS({REG_NAMES[index]})", None
        if mi.b == 0b011:
            index = self.ir & 0x3
            return self.regs[index], f"RD({REG_NAMES[index]})", None
        if mi.b == 0b100:
            return self.regs[2], "RI(R2)", None
        if mi.b == 0b101:
            return self.sp, "SP(R3)", None
        if mi.b == 0b110:
            return self.pc, "PC", None
        if inta_vector is not None:
            return inta_vector, "8259_VECTOR", None
        return None, None, None

    def _pending_alu_flags(self, mi: MicroInstruction, bus_source: Optional[str]) -> Dict[str, int]:
        if bus_source != "ALU":
            return {}
        _, flags = self._alu(mi)
        return flags

    def _alu(self, mi: MicroInstruction) -> Tuple[int, Dict[str, int]]:
        a = self.reg_a & BYTE_MASK
        b = self.reg_b & BYTE_MASK
        s = mi.s
        # The textbook labels bit 23 as M23 in the microinstruction table.
        # TODO: confirm with the physical wiring whether M23 is the ALU CN input.
        cn = mi.m23 & 1
        result = a
        flags: Dict[str, int] = {}
        if s == 0b0000:
            result = a
        elif s == 0b0001:
            result = b
        elif s == 0b0010:
            result = a & b
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b0011:
            result = a | b
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b0100:
            result = (~a) & BYTE_MASK
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b0101:
            count = b & 0x7
            result = _ror(a, count)
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b0110:
            if cn:
                result = ((self.fc & 1) << 7) | (a >> 1)
                flags["fc"] = a & 1
            else:
                result = a >> 1
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b0111:
            if cn:
                result = ((a << 1) & BYTE_MASK) | (self.fc & 1)
                flags["fc"] = (a >> 7) & 1
            else:
                result = (a << 1) & BYTE_MASK
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b1000:
            result = a
            flags["fc"] = cn
        elif s == 0b1001:
            total = a + b
            result = total & BYTE_MASK
            flags["fc"] = 1 if total > BYTE_MASK else 0
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b1010:
            total = a + b + (self.fc & 1)
            result = total & BYTE_MASK
            flags["fc"] = 1 if total > BYTE_MASK else 0
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b1011:
            total = a - b
            result = total & BYTE_MASK
            flags["fc"] = 1 if total < 0 else 0
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b1100:
            total = a - 1
            result = total & BYTE_MASK
            flags["fc"] = 1 if total < 0 else 0
            flags["fz"] = 1 if result == 0 else 0
        elif s == 0b1101:
            total = a + 1
            result = total & BYTE_MASK
            flags["fc"] = 1 if total > BYTE_MASK else 0
            flags["fz"] = 1 if result == 0 else 0
        return result & BYTE_MASK, flags

    def _load_a_field(self, mi: MicroInstruction, bus: int) -> None:
        value = bus & BYTE_MASK
        if mi.a == 0b001:
            self.reg_a = value
        elif mi.a == 0b010:
            self.reg_b = value
        elif mi.a == 0b011:
            self.regs[self.ir & 0x3] = value
        elif mi.a == 0b100:
            self.sp = value
        elif mi.a == 0b110:
            self.ar = value
        elif mi.a == 0b111:
            self.ir = value

    def _write_target(self, mi: MicroInstruction, bus: int, io_event: Optional[str]) -> Optional[str]:
        if mi.iom:
            event = self._write_io(self.ar, bus)
            return f"{io_event}; {event}" if io_event else event
        self.mem[self.ar] = bus & BYTE_MASK
        event = f"MEM[{self.ar:02X}]<-{bus & BYTE_MASK:02X}"
        return f"{io_event}; {event}" if io_event else event

    def _read_io(self, port: int) -> Tuple[int, str]:
        port &= BYTE_MASK
        zone = port >> 6
        if zone == 0:
            return self.in_unit, f"IN[{port:02X}]->{self.in_unit:02X}"
        if zone == 3:
            value, event = self.pic.read(port)
            return value, event
        if zone == 1:
            return self.out_unit, f"OUT[{port:02X}]->{self.out_unit:02X}"
        return 0, f"8253/unused[{port:02X}]->00"

    def _write_io(self, port: int, value: int) -> str:
        port &= BYTE_MASK
        value &= BYTE_MASK
        zone = port >> 6
        if zone == 1:
            self.out_unit = value
            return f"OUT[{port:02X}]<-{value:02X}"
        if zone == 3:
            return self.pic.write(port, value)
        if zone == 0:
            return f"IN switches[{port:02X}] ignore write {value:02X}"
        return f"8253/unused[{port:02X}]<-{value:02X}"

    def _next_micro_addr(self, mi: MicroInstruction) -> int:
        ma = mi.ma & MICRO_NEXT_MASK
        if mi.c == 0b001:
            op_hi2 = (self.ir >> 6) & 0x3
            if op_hi2 != 0b11:
                return ((ma & 0x30) | ((self.ir >> 4) & 0x0F)) & MICRO_NEXT_MASK
            return ((ma & 0x30) | 0x0C | ((self.ir >> 2) & 0x03)) & MICRO_NEXT_MASK
        if mi.c == 0b010:
            return ((ma & 0x3C) | ((self.ir >> 4) & 0x03)) & MICRO_NEXT_MASK
        if mi.c == 0b011:
            flag = 1 if (self.fc or self.fz) else 0
            return ((ma & 0x2F) | (flag << 4)) & MICRO_NEXT_MASK
        if mi.c == 0b100:
            flag = 1 if (self.intr and self.ei) else 0
            return ((ma & 0x1F) | (flag << 5)) & MICRO_NEXT_MASK
        return ma


def _ror(value: int, count: int) -> int:
    value &= BYTE_MASK
    count &= 7
    if count == 0:
        return value
    return ((value >> count) | (value << (8 - count))) & BYTE_MASK


def _format_changes(before: Dict[str, int], after: Dict[str, int]) -> List[str]:
    order = ["PC", "IR", "AR", "A", "B", "R0", "R1", "R2", "R3", "FZ", "FC", "EI", "OUT", "uAR", "INTR"]
    changes: List[str] = []
    for key in order:
        if before.get(key) != after.get(key):
            if key in {"FZ", "FC", "EI", "INTR"}:
                changes.append(f"{key}:{before[key]}->{after[key]}")
            else:
                changes.append(f"{key}:{before[key]:02X}->{after[key]:02X}")
    return changes


def disassemble(ir: int, operand: Optional[int] = None) -> str:
    ir &= BYTE_MASK
    op = (ir >> 4) & 0xF
    rs = (ir >> 2) & 0x3
    rd = ir & 0x3
    imm = "??" if operand is None else f"{operand & BYTE_MASK:02X}"
    if op == 0x0:
        return f"ADD {REG_NAMES[rd]},{REG_NAMES[rs]}"
    if op == 0x1:
        return f"AND {REG_NAMES[rd]},{REG_NAMES[rs]}"
    if op == 0x2:
        return f"IN {REG_NAMES[rd]},{imm}H"
    if op == 0x3:
        return f"OUT {imm}H,{REG_NAMES[rs]}"
    if op == 0x4:
        return f"MOV {REG_NAMES[rd]},{REG_NAMES[rs]}"
    if op == 0x5:
        return "HALT"
    if op == 0x6:
        return f"LDI {REG_NAMES[rd]},{imm}H"
    if op == 0x7:
        return "STI"
    if op == 0x8:
        return "CLI"
    if op == 0x9:
        return f"PUSH {REG_NAMES[rs]}"
    if op == 0xA:
        return f"POP {REG_NAMES[rd]}"
    if op == 0xB:
        return "IRET"
    mode = ["D", "[D]", "RI+D", "PC+D"][(ir >> 2) & 0x3]
    if op == 0xC:
        return f"LAD {mode},{REG_NAMES[rd]} ; D={imm}H"
    if op == 0xD:
        return f"STA {mode},{REG_NAMES[rd]} ; D={imm}H"
    if op == 0xE:
        return f"JMP {mode} ; D={imm}H"
    if op == 0xF:
        return f"BZC {mode} ; D={imm}H"
    return f"DB {ir:02X}"
