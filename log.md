第一个问题：从站3（右上角的驱动板）容易掉线


[Sat Apr 25 19:52:41 2026] EtherCAT ERROR 0-2: Reception of CoE upload response failed: No response.
[Sat Apr 25 19:52:41 2026] EtherCAT ERROR 0-2: Failed to process SDO request.
[Sat Apr 25 19:52:41 2026] EtherCAT ERROR 0-2: Reception of CoE upload response failed: No response.
[Sat Apr 25 19:52:41 2026] EtherCAT ERROR 0-2: Failed to process SDO request.
[Sat Apr 25 19:52:50 2026] EtherCAT 0: 2 slave(s) responding on main device. Re-scanning on next possibility.
[Sat Apr 25 19:52:50 2026] EtherCAT 0: Re-scanning now.
[Sat Apr 25 19:52:50 2026] EtherCAT 0: Scanning bus.

这里可以看到从站3直接无法没有回复主战的消息

第2个问题


ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP   +  0x0000029c:0x03831002
1  0:1  OP   +  0x00000000:0x00000000
2  0:2  ???  +  0x00000000:0x00000000
3  0:3  ???  +  0x00000000:0x00000000
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP   +  0x00000000:0x00000000
1  0:1  ???  +  0x00000000:0x00000000
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP  +  0x0000029c:0x03831002
1  0:1  OP  +  0x0000029c:0x03831002
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP   +  0x00000000:0x00000000
1  0:1  ???  +  0x00000000:0x00000000
2  0:2  ???  +  0x00000000:0x00000000
3  0:3  ???  +  0x00000000:0x00000000
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP   +  0x0000029c:0x03831002
1  0:1  OP   +  0x0000029c:0x03831002
2  0:2  ???  +  0x00000000:0x00000000
3  0:3  ???  +  0x00000000:0x00000000
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP  +  0x0000029c:0x03831002
1  0:1  OP  +  0x00000000:0x00000000
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP   E  0x00000000:0x00000000
1  0:1  OP   +  0x00000000:0x00000000
2  0:2  ???  +  0x00000000:0x00000000
3  0:3  ???  +  0x00000000:0x00000000
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP   +  0x0000029c:0x03831002
1  0:1  ???  +  0x00000000:0x00000000
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP     +  0x0000029c:0x03831002
1  0:1  OP     +  0x0000029c:0x03831002
2  0:2  PREOP  +  0x0000029c:0x03831002
3  0:3  PREOP  +  0x0000029c:0x03831002
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP     +  0x0000029c:0x03831002
1  0:1  OP     +  0x0000029c:0x03831002
2  0:2  PREOP  +  0x0000029c:0x03831002
3  0:3  PREOP  +  0x0000029c:0x03831002
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ sudo /opt/etherlab/bin/ethercat slaves
0  0:0  OP     +  0x0000029c:0x03831002
1  0:1  OP     +  0x0000029c:0x03831002
2  0:2  OP     +  0x0000029c:0x03831002
3  0:3  PREOP  +  0x0000029c:0x03831002
ihear1@ihear1-HP-ProDesk-600-G3-PCI-MT:~$ 


有个很难的问题使用ethercat slaves能够看到4个处于正常状态：PREOP，OP的从站，但是当一个电机使能后，在Ethercat连线的下游的从站会掉线，虽然后续会重现上线，但会出现无法使能的情况——status反复跳动，就是上不去使能。
解决方案：最后发现是其中的一条线有问题，虽然能检测到从站，但不代表线的通讯是OK的。