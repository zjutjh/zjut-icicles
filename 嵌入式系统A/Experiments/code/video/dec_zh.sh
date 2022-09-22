#!/bin/sh
echo -n 1 > /sys/module/dm365_imp/parameters/oper_mode

rmmod cmemk 2>/dev/null
rmmod irqk 2>/dev/null
rmmod edmak 2>/dev/null
rmmod dm365mmap 2>/dev/null
  
insmod cmemk.ko phys_start=0x86000000 phys_end=0x88000000 \
                pools=1x384,29x56,6x1024,1x2688,1x3328,1x8704,1x10240,1x13184,1x26224,1x48396,4x460800,1x158368,2x282624,2x60825,20x718848,2x2697152,1x6045696 allowOverlap=1
                phys_start_1=0x00001000 phys_end_1=0x00008000 pools_1=1x28672
              
insmod irqk.ko
insmod edmak.ko
insmod dm365mmap.ko
sleep 3
                 

DMAI_DEBUG=2 decode_zhfinal -y 2 -v /mnt/mmc/video/2019-01-12-12-14-19.264 -O lcd  > decode_D1.log &

