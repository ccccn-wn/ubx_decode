"""
pygnssutils - gnssapp.py

*** FOR ILLUSTRATION ONLY - NOT FOR PRODUCTION USE ***

Skeleton GNSS application which continuously receives, parses and prints
NMEA, UBX or RTCM data from a receiver until the stop Event is set or
stop() method invoked. Assumes receiver is connected via serial USB or UART1 port.

The app also implements basic methods needed by certain pygnssutils classes.

Optional keyword arguments:

- sendqueue - any data placed on this Queue will be sent to the receiver
  (e.g. UBX commands/polls or NTRIP RTCM data). Data must be a tuple of 
  (raw_data, parsed_data).
- idonly - determines whether the app prints out the entire parsed message,
  or just the message identity.
- enableubx - suppresses NMEA receiver output and substitutes a minimum set
  of UBX messages instead (NAV-PVT, NAV-SAT, NAV-DOP, RXM-RTCM).
- showhacc - show estimate of horizonal accuracy in metres (if available).

Created on 27 Jul 2023

:author: semuadmin
:copyright: SEMU Consulting © 2023
:license: BSD 3-Clause
"""
# pylint: disable=invalid-name, too-many-instance-attributes

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from queue import Empty, Queue
from threading import Event, Thread
from time import sleep

from pynmeagps import NMEAMessageError, NMEAParseError
from pyrtcm import RTCMMessage, RTCMMessageError, RTCMParseError
from serial import Serial

from pyubx2 import (
    NMEA_PROTOCOL,
    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
)
import pandas as pd
import numpy as np
import time
import struct
CONNECTED = 1

def bytes_to_Doublefloat(bytes):
    return struct.unpack(f'<d', bytes)[0]
def bytes_to_float(bytes):
    return struct.unpack(f'<f', bytes)[0]

def bytes_to_U32(bytes):
    return struct.unpack(f'<I', bytes)[0]

def bytes_to_U16(bytes):
    return struct.unpack(f'<H', bytes)[0]

def bytes_to_U8(bytes):
    return struct.unpack(f'B', bytes)[0]


class GNSSSkeletonApp:
    """
    Skeleton GNSS application which communicates with a GNSS receiver.
    """

    def __init__(
        self, port: str, baudrate: int, timeout: float, stopevent: Event, **kwargs
    ):
        """
        Constructor.

        :param str port: serial port e.g. "/dev/ttyACM1"
        :param int baudrate: baudrate
        :param float timeout: serial timeout in seconds
        :param Event stopevent: stop event
        """

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.stopevent = stopevent
        self.sendqueue = kwargs.get("sendqueue", None)
        self.idonly = kwargs.get("idonly", True)
        self.enableubx = kwargs.get("enableubx", False)
        self.showhacc = kwargs.get("showhacc", False)
        self.stream = None
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.sep = 0

    def __enter__(self):
        """
        Context manager enter routine.
        """

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit routine.

        Terminates app in an orderly fashion.
        """

        self.stop()

    def run(self):
        """
        Run GNSS reader/writer.
        """

        self.enable_ubx(self.enableubx)

        self.stream = Serial(self.port, self.baudrate, timeout=self.timeout)
        self.stopevent.clear()

        read_thread = Thread(
            target=self._read_loop,
            args=(
                self.stream,
                self.stopevent,
                self.sendqueue,
            ),
            daemon=True,
        )
        read_thread.start()

    def stop(self):
        """
        Stop GNSS reader/writer.
        """

        self.stopevent.set()
        if self.stream is not None:
            self.stream.close()

    def _read_loop(self, stream: Serial, stopevent: Event, sendqueue: Queue):
        """
        THREADED
        Reads and parses incoming GNSS data from the receiver,
        and sends any queued output data to the receiver.

        :param Serial stream: serial stream
        :param Event stopevent: stop event
        :param Queue sendqueue: queue for messages to send to receiver
        """

        ubr = UBXReader(
            stream, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL | RTCM3_PROTOCOL)
        )
        svID = []
        cpMes = []
        CN0 = []
        prMes = []
        rcvTow = []
        rcvWeek = []
        BeiJingTime = []
        DoppleMes = []
        RunCnt = 0
        # while not stopevent.is_set():
        while RunCnt < 10:
            try:
                raw_data = ubr.read()
                if raw_data != None:
                    RunCnt += 1
                    lenm = len(raw_data)
                    hdr = raw_data[0:2]
                    clsid = raw_data[2:3]
                    msgid = raw_data[3:4]
                    lenb = raw_data[4:6]
                    if lenb == b"\x00\x00":
                        payload = None
                        leni = 0
                    else:
                        payload = raw_data[6 : lenm - 2]
                    leni = len(payload)
                    numMeasCal = (leni - 16)/32
                    tow = bytes_to_Doublefloat(payload[0:8])
                    week = bytes_to_U16(payload[8:10])
                    numMeasRcv = bytes_to_U8(payload[11:12])
                    if numMeasCal == numMeasRcv: #整数
                        for i in range(numMeasRcv):
                            BeiJingTime.append(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()))
                            rcvTow.append(tow)
                            prm = bytes_to_Doublefloat(payload[16+32*i:24+32*i])
                            prMes.append(prm)
                            cpm = bytes_to_Doublefloat(payload[24+32*i : 32+32*i])
                            cpMes.append(cpm)            
                            svid = bytes_to_U8(payload[37+32*i :38+32*i])
                            svID.append(svid)
                            cn0 = bytes_to_U8(payload[42+32*i:43+32*i])
                            CN0.append(cn0)
                            # dpMes = bytes_to_float(payload[42+32*i:43+32*i])

                self._send_data(ubr.datastream, sendqueue)

            except (
                UBXMessageError,
                UBXParseError,
                NMEAMessageError,
                NMEAParseError,
                RTCMMessageError,
                RTCMParseError,
            ) as err:
                print(f"Error parsing data stream {err}")
                continue
        # writer = pd.ExcelWriter('output.xlsx')
        # df1 = pd.DataFrame({'TOW': rcvTow})
        # df2 = pd.DataFrame({'svid': svID})
        # df3 = pd.DataFrame({'cpMes': cpMes})

        # df1.to_excel(writer,'Sheet1')
        # df2.to_excel(writer,'Sheet2')
        # df3.to_excel(writer,'Sheet3')

        # writer.close()

        df = pd.DataFrame({'BeiJingTime': BeiJingTime,'rcvTow':rcvTow ,'AdrM':cpMes ,'PrM':prMes ,'CN0':CN0 ,'svid': svID})
        df.to_excel('test1.xlsx', sheet_name='sheet1', index=False)
        print("\n save successfully \n")
    def _extract_coordinates(self, parsed_data: object):
        """
        Extract current navigation solution from NMEA or UBX message.

        :param object parsed_data: parsed NMEA or UBX navigation message
        """

        if hasattr(parsed_data, "lat"):
            self.lat = parsed_data.lat
        if hasattr(parsed_data, "lon"):
            self.lon = parsed_data.lon
        if hasattr(parsed_data, "alt"):
            self.alt = parsed_data.alt
        if hasattr(parsed_data, "hMSL"):  # UBX hMSL is in mm
            self.alt = parsed_data.hMSL / 1000
        if hasattr(parsed_data, "sep"):
            self.sep = parsed_data.sep
        if hasattr(parsed_data, "hMSL") and hasattr(parsed_data, "height"):
            self.sep = (parsed_data.height - parsed_data.hMSL) / 1000
        if self.showhacc and hasattr(parsed_data, "hAcc"):  # UBX hAcc is in mm
            unit = 1 if parsed_data.identity == "PUBX00" else 1000
            print(f"Estimated horizontal accuracy: {(parsed_data.hAcc / unit):.3f} m")

    def _send_data(self, stream: Serial, sendqueue: Queue):
        """
        Send any queued output data to receiver.
        Queue data is tuple of (raw_data, parsed_data).

        :param Serial stream: serial stream
        :param Queue sendqueue: queue for messages to send to receiver
        """

        if sendqueue is not None:
            try:
                while not sendqueue.empty():
                    data = sendqueue.get(False)
                    raw, parsed = data
                    source = "NTRIP>>" if isinstance(parsed, RTCMMessage) else "GNSS<<"
                    if self.idonly:
                        print(f"{source} {parsed.identity}")
                    else:
                        print(parsed)
                    stream.write(raw)
                    sendqueue.task_done()
            except Empty:
                pass

    def enable_ubx(self, enable: bool):
        """
        Enable UBX output and suppress NMEA.

        :param bool enable: enable UBX and suppress NMEA output
        """

        layers = 1
        transaction = 0
        cfg_data = []
        for port_type in ("USB", "UART1"):
            cfg_data.append((f"CFG_{port_type}OUTPROT_NMEA", not enable))
            cfg_data.append((f"CFG_{port_type}OUTPROT_UBX", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port_type}", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_SAT_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_DOP_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_RXM_RTCM_{port_type}", enable))

        msg = UBXMessage.config_set(layers, transaction, cfg_data)
        self.sendqueue.put((msg.serialize(), msg))

    def get_coordinates(self) -> tuple:
        """
        Return current receiver navigation solution.
        (method needed by certain pygnssutils classes)

        :return: tuple of (connection status, lat, lon, alt and sep)
        :rtype: tuple
        """

        return (CONNECTED, self.lat, self.lon, self.alt, self.sep)

    def set_event(self, eventtype: str):
        """
        Create event.
        (stub method needed by certain pygnssutils classes)

        :param str eventtype: name of event to create
        """

        # create event of specified eventtype

def GnssRawData_Decode(parse):
    parse
    

if __name__ == "__main__":
    arp = ArgumentParser(
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    arp.add_argument(
        "-P", "--port", required=False, help="Serial port", default="COM13"
    )
    arp.add_argument(
        "-B", "--baudrate", required=False, help="Baud rate", default=921600, type=int
    )
    arp.add_argument(
        "-T", "--timeout", required=False, help="Timeout in secs", default=3, type=float
    )

    args = arp.parse_args()
    send_queue = Queue()
    stop_event = Event()

    try:
        print("Starting GNSS reader/writer...\n")
        with GNSSSkeletonApp(
            args.port,
            int(args.baudrate),
            float(args.timeout),
            stop_event,
            sendqueue=send_queue,
            idonly=False,
            enableubx=True,
            showhacc=True,
        ) as gna:
            gna.run()
            while True:
                sleep(1)

    except KeyboardInterrupt:
        stop_event.set()
        print("Terminated by user")
