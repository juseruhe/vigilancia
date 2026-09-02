Es un receptor LoRa 433 MHz para NuttX sobre Raspberry Pi Pico (RP2040). Es el port de un sketch Arduino: recibe mensajes de una Raspberry Pi 4
vía módulo SX1278 (Ra-02), los muestra en un OLED SSD1306 128x32 y devuelve un ACK por radio a cada paquete.

```mermaid

flowchart TB
 subgraph OLEDPATH["update_display() → SSD1306"]
    direction LR
        T1["y=0  RX%u LOST%u"]
        FB["ssd1306_clear<br>g_fb512"]
        T2["y=11 RSSI%d SNR%s"]
        T3["y=22 ultimo mensaje"]
        FLUSH["ssd1306_flush<br>ventana 0x21/0x22<br>16 bloques de 32B con prefijo 0x40"]
  end
    START(["main"]) --> OLED{"ssd1306_open<br>/dev/i2c0 @0x3c"}
    OLED -- error --> FAIL1["stderr + EXIT_FAILURE"]
    OLED -- ok --> SPLASH["display_splash<br>Pico LoRa RX 1.5s"]
    SPLASH --> RADIO{"sx1278_open /dev/spi0<br>valida REG_VERSION == 0x12"}
    RADIO -- "-ENODEV" --> FAIL2["display_error<br>SX1278 fallo<br>ssd1306_close + EXIT_FAILURE"]
    RADIO -- ok --> CFG["sx1278_config"]
    CFG --> CFGD["SLEEP + LoRa mode<br>FRF = 433MHz<br>FIFO base TX/RX = 0<br>LNA boost<br>MC3: AGC auto<br>MC1: BW7 | CR4/5 | hdr explicito<br>MC2: SF7 | CRC<br>preambulo 8 sym<br>sync 0x12<br>PA_BOOST 17dBm, OCP<br>→ STDBY"]
    CFGD --> RX["sx1278_receive_mode<br>FIFO_ADDR_PTR=0<br>MODE_RX_CONTINUOUS"]
    RX --> INIT["msg=Esperando Pi4<br>update_display<br>lastrx = now_ms"]
    INIT --> LOOP{{"bucle infinito"}}
    LOOP --> PARSE["sx1278_parse_packet"]
    PARSE --> IRQ{"REG_IRQ_FLAGS<br>RxDone 0x40?"}
    IRQ -- no --> WD{"hubo RX previo y<br>now - lastrx &gt; 6000 ms?"}
    IRQ -- si --> CLR["limpia IRQ write-1-to-clear"]
    CLR --> CRC{"PayloadCrcError<br>0x20?"}
    CRC -- si --> BAD["return -EBADMSG<br>printf CRC invalido"]
    BAD --> WD
    CRC -- no --> READ["len = REG_RX_NB_BYTES<br>RSSI = raw - 164<br>SNR = int8 q4<br>FIFO_ADDR_PTR = RX_CURRENT_ADDR<br>burst SPI de len+1 bytes"]
    READ --> UPD["g_rxcount++<br>lastrx = now_ms<br>printf + update_display"]
    UPD --> ACK["sx1278_send Pico->Pi4 ACK #n"]
    ACK --> TX["STDBY, limpia IRQ 0xff<br>escribe FIFO + PAYLOAD_LENGTH<br>MODE_TX<br>poll TxDone cada 1ms max 3s"]
    TX --> TXR{"TxDone?"}
    TXR -- si --> BACKRX["limpia TxDone<br>sx1278_receive_mode"]
    TXR -- timeout --> TXERR["-ETIMEDOUT<br>vuelve a RX igualmente<br>printf, no aborta"]
    TXERR --> BACKRX
    BACKRX --> UPD2["update_display"]
    UPD2 --> WD
    WD -- si --> LOST["g_lostcount++<br>lastrx = now_ms<br>msg = Sin senal<br>update_display"]
    WD -- no --> SLEEP["usleep 2000"]
    LOST --> SLEEP
    SLEEP --> LOOP
    FB --> T1
    T1 --> T2
    T2 --> T3
    T3 --> FLUSH
    UPD -.-> OLEDPATH
     START:::initNode
     OLED:::initNode
     FAIL1:::errorNode
     SPLASH:::displayNode
     RADIO:::initNode
     FAIL2:::errorNode
     CFG:::initNode
     LOOP:::loopNode
     IRQ:::loopNode
     WD:::loopNode
     BAD:::errorNode
     UPD:::dataNode
     ACK:::dataNode
     TXR:::loopNode
     TXERR:::errorNode
     UPD2:::dataNode
     LOST:::dataNode
     OLEDPATH:::displayNode
    classDef errorNode stroke:#f87171,fill:#fef2f2
    classDef initNode stroke:#38bdf8,fill:#f0f9ff
    classDef loopNode stroke:#a78bfa,fill:#f5f3ff
    classDef dataNode stroke:#4ade80,fill:#f0fdf4
    classDef displayNode stroke:#2dd4bf,fill:#f0fdfa
    classDef loopNode stroke:#a78bfa,fill:#f5f3ff
    classDef dataNode stroke:#4ade80,fill:#f0fdf4
    classDef displayNode stroke:#2dd4bf,fill:#f0fdfa
    ```
