import sys
import os
import functools
import datetime
import time
import struct
import threading
import warnings
import math

# 判断多次运行
try:
    cdb_first_run
except Exception:
    cdb_first_run = False
else:
    print("暂不支持重复运行，若不小心关闭了gui界面，请重启python内核！")
    raise RuntimeError('暂不支持重复运行')


# 屏蔽一些警告，开始堆屎
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=UserWarning)

from canlib import canlib, Frame
from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt

# 初始化库
from cdb_dstruct_init import *

# 初始化界面
import mainWin_ui

VERSION_STR = "CDB_GUI V0.2"


class mainWindow(QtWidgets.QMainWindow, mainWin_ui.Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.setWindowTitle(VERSION_STR)

        self.actioncdb_GUI.triggered.connect(functools.partial(
            QtWidgets.QMessageBox.aboutQt, None, "about cdb_GUI"))

        self.statusBar().showMessage(VERSION_STR, 2000)

        # paraSet 界面部分
        # 补全器
        self.completer = QtWidgets.QCompleter(list(varDict.keys()))
        self.completer.setFilterMode(QtCore.Qt.MatchContains)
        self.completer.setCaseSensitivity(QtCore.Qt.CaseInsensitive)

        self.__paraTabInitUtil(self.lineEdit, self.lineEditV,
                               self.pushButtonR, self.pushButtonW)
        self.__paraTabInitUtil(self.lineEdit_2, self.lineEditV_2,
                               self.pushButtonR_2, self.pushButtonW_2)
        self.__paraTabInitUtil(self.lineEdit_3, self.lineEditV_3,
                               self.pushButtonR_3, self.pushButtonW_3)
        self.__paraTabInitUtil(self.lineEdit_4, self.lineEditV_4,
                               self.pushButtonR_4, self.pushButtonW_4)
        self.__paraTabInitUtil(self.lineEdit_5, self.lineEditV_5,
                               self.pushButtonR_5, self.pushButtonW_5)
        self.__paraTabInitUtil(self.lineEdit_6, self.lineEditV_6,
                               self.pushButtonR_6, self.pushButtonW_6)
        self.__paraTabInitUtil(self.lineEdit_7, self.lineEditV_7,
                               self.pushButtonR_7, self.pushButtonW_7)
        self.__paraTabInitUtil(self.lineEdit_8, self.lineEditV_8,
                               self.pushButtonR_8, self.pushButtonW_8)
        self.__paraTabInitUtil(self.lineEdit_9, self.lineEditV_9,
                               self.pushButtonR_9, self.pushButtonW_9)

        self.lineEdit.setText(paraSetPreDefName[0])
        self.lineEdit_2.setText(paraSetPreDefName[1])
        self.lineEdit_3.setText(paraSetPreDefName[2])
        self.lineEdit_4.setText(paraSetPreDefName[3])
        self.lineEdit_5.setText(paraSetPreDefName[4])
        self.lineEdit_6.setText(paraSetPreDefName[5])
        self.lineEdit_7.setText(paraSetPreDefName[6])
        self.lineEdit_8.setText(paraSetPreDefName[7])
        self.lineEdit_9.setText(paraSetPreDefName[8])

        # trig 界面部分
        self.lineEdit_S.setText(trigSrcPreDefName[0])
        self.lineEdit_S_2.setText(trigSrcPreDefName[1])
        self.lineEdit_S.setCompleter(self.completer)
        self.lineEdit_S_2.setCompleter(self.completer)

        self.lineEdit_L.setText(logSrcPreDefName[0])
        self.lineEdit_L_2.setText(logSrcPreDefName[1])
        self.lineEdit_L_3.setText(logSrcPreDefName[2])
        self.lineEdit_L_4.setText(logSrcPreDefName[3])
        self.lineEdit_L_5.setText(logSrcPreDefName[4])
        self.lineEdit_L_6.setText(logSrcPreDefName[5])
        self.lineEdit_L_7.setText(logSrcPreDefName[6])
        self.lineEdit_L_8.setText(logSrcPreDefName[7])
        self.lineEdit_L_9.setText(logSrcPreDefName[8])
        self.lineEdit_L_10.setText(logSrcPreDefName[9])
        self.lineEdit_L_11.setText(logSrcPreDefName[10])
        self.lineEdit_L_12.setText(logSrcPreDefName[11])
        self.lineEdit_L_13.setText(logSrcPreDefName[12])
        self.lineEdit_L_14.setText(logSrcPreDefName[13])
        self.lineEdit_L_15.setText(logSrcPreDefName[14])
        self.lineEdit_L_16.setText(logSrcPreDefName[15])
        self.lineEdit_L.setCompleter(self.completer)
        self.lineEdit_L_2.setCompleter(self.completer)
        self.lineEdit_L_3.setCompleter(self.completer)
        self.lineEdit_L_4.setCompleter(self.completer)
        self.lineEdit_L_5.setCompleter(self.completer)
        self.lineEdit_L_6.setCompleter(self.completer)
        self.lineEdit_L_7.setCompleter(self.completer)
        self.lineEdit_L_8.setCompleter(self.completer)
        self.lineEdit_L_9.setCompleter(self.completer)
        self.lineEdit_L_10.setCompleter(self.completer)
        self.lineEdit_L_11.setCompleter(self.completer)
        self.lineEdit_L_12.setCompleter(self.completer)
        self.lineEdit_L_13.setCompleter(self.completer)
        self.lineEdit_L_14.setCompleter(self.completer)
        self.lineEdit_L_15.setCompleter(self.completer)
        self.lineEdit_L_16.setCompleter(self.completer)

        self.pushButton_np.clicked.connect(self.wait4TrigSlot)
        self.pushButton_trigCfg.clicked.connect(self.trigCfgSlot)

        self.lineEdit_buffSize.setText(buffSize_init)
        self.lineEdit_trigtime.setText(trigTimeTar_init)

        # dump 界面部分

        self.gridLayout_draw = QtWidgets.QGridLayout(self.groupBox_draw)
        self.gridLayout_draw.setObjectName("gridLayout_draw")

        self.drawCheckBoxList = []

        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)

        for ii in range(4):
            for jj in range(4):
                temp = QtWidgets.QCheckBox(self.groupBox_draw)
                temp.setObjectName("checkBox_"+str(ii*4+jj))
                temp.setText("checkBox_"+str(ii*4+jj))
                self.gridLayout_draw.addWidget(temp, ii, jj, 1, 1)
                temp.setSizePolicy(sizePolicy)
                self.drawCheckBoxList.append(temp)

        self.pushButton_dCfg.clicked.connect(self.dumpCfgSlot)
        self.pushButton_dump.clicked.connect(self.dumpSlot)
        self.pushButton_cb.clicked.connect(self.toCbSlot)
        self.pushButton_setAll.clicked.connect(self.setAllSlot)
        self.pushButton_clearAll.clicked.connect(self.clrAllSlot)
        self.pushButton_draw.clicked.connect(self.drawSlot)

        self.lineEdit_sAddr.setText(startAddr_init)
        self.lineEdit_dSize.setText(dumpSize_init)
        self.lineEdit_isrFreq.setText(IsrFreq_init)

    def dumpCfgSlot(self):
        global startAddr
        global dumpSize
        global endAddr
        global totalCycle
        global logTarCnt

        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_sAddr.text(), None, loc)
            startAddr = int(loc['val'])
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" startAddr err!", file=sys.stderr)
            self.statusBar().showMessage("startAddr err!", 1000)
            return

        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_dSize.text(), None, loc)
            dumpSize = int(loc['val'])
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" dumpSize err!", file=sys.stderr)
            self.statusBar().showMessage("dumpSize err!", 1000)
            return

        endAddr = startAddr+2*dumpSize
        totalCycle = int(dumpSize/logTarCnt)

        global recvDict
        recvDict = {}

        global logTarNameList
        for ii in range(16):
            self.drawCheckBoxList[ii].setText(str(ii)+"-N/A")

        for ii in range(logTarCnt):
            if (logTarNameList[ii] != ""):
                self.drawCheckBoxList[ii].setText(
                    str(ii)+"-"+logTarNameList[ii])
            else:
                break

        self.statusBar().showMessage("dump cfg ok!", 1000)

    def dumpSlot(self):
        if(dumpBuff()):
            return

        global totalCycle
        global logTarCnt
        global logTarNameList
        global valYnpTab

        valYnpTab = []

        QtWidgets.QApplication.processEvents()

        try:
            for ii in range(logTarCnt):
                valYnpTab.append(np.zeros(totalCycle))
                valType = varTypeDict.get(logTarNameList[ii], "float")

                if (valType == "float"):
                    for jj in range(totalCycle):
                        valYnpTab[ii][jj] = struct.unpack(
                            "<f", recvDict[(jj*logTarCnt+ii)*2])[0]
                elif (valType == "uint32_t"):
                    for jj in range(totalCycle):
                        valYnpTab[ii][jj] = struct.unpack(
                            "<I", recvDict[(jj*logTarCnt+ii)*2])[0]
                elif (valType == "int32_t"):
                    for jj in range(totalCycle):
                        valYnpTab[ii][jj] = struct.unpack(
                            "<i", recvDict[(jj*logTarCnt+ii)*2])[0]
                elif (valType == "uint16_t"):
                    for jj in range(totalCycle):
                        valYnpTab[ii][jj] = struct.unpack(
                            "<H", recvDict[(jj*logTarCnt+ii)*2][0:2])[0]
                elif (valType == "int16_t"):
                    for jj in range(totalCycle):
                        valYnpTab[ii][jj] = struct.unpack(
                            "<h", recvDict[(jj*logTarCnt+ii)*2][0:2])[0]
                else:
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" decoding {} type err!".format(logTarNameList[ii]), file=sys.stderr)
                    self.statusBar().showMessage(
                        "decoding {} type err!".format(logTarNameList[ii]), 1000)
                    break
        except Exception:
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" decoding err2!", file=sys.stderr)
                self.statusBar().showMessage(
                    "decoding err2!", 1000)


    def toCbSlot(self):
        global cbStr
        global totalCycle
        global valYnpTab
        global logTarCnt
        global timex

        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_isrFreq.text(), None, loc)
            IsrFreq = loc['val']
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" IsrFreq err!", file=sys.stderr)
            self.statusBar().showMessage("IsrFreq err!", 1000)
            return

        timex = np.arange(totalCycle)/IsrFreq

        cbStr = ""

        for ii in range(totalCycle):
            cbStr += str(timex[ii])
            for jj in range(logTarCnt):
                cbStr += "\t"
                cbStr += str(valYnpTab[jj][ii])
            cbStr += "\n"

        clipBoard = QtWidgets.QApplication.clipboard()
        clipBoard.setText(cbStr)

        self.statusBar().showMessage("to clipBoard fin!", 1000)

    def setAllSlot(self):
        for ii in range(16):
            self.drawCheckBoxList[ii].setChecked(True)

    def clrAllSlot(self):
        for ii in range(16):
            self.drawCheckBoxList[ii].setChecked(False)

    def drawSlot(self):
        global totalCycle
        global valYnpTab
        global logTarCnt
        global timex

        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_isrFreq.text(), None, loc)
            IsrFreq = loc['val']
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" IsrFreq err!", file=sys.stderr)
            self.statusBar().showMessage("IsrFreq err!", 1000)
            return
        timex = np.arange(totalCycle)/IsrFreq
        # 画图分析
        plt.figure(num=self.lineEdit_figTitle.text() + datetime.datetime.now().strftime(' @ [%Y-%m-%d, %H:%M:%S]'))

        for ii in range(logTarCnt):
            if (self.drawCheckBoxList[ii].checkState()):
                plt.plot(timex, valYnpTab[ii], label=logTarNameList[ii])

        plt.xlabel("time:s")
        plt.ylabel("value")
        plt.title(self.lineEdit_figTitle.text())
        plt.legend()
        plt.show()

        self.statusBar().showMessage("plot fin!", 1000)

    def trigCfgSlot(self):
        global trigSrcNameList
        global trigSrcAddrList
        global varDict
        trigSrcNameList = []
        trigSrcNameList.append(self.lineEdit_S.text())
        trigSrcNameList.append(self.lineEdit_S_2.text())

        try:
            trigSrcAddrList = [varDict[trigSrcNameX] if trigSrcNameX != "" else 0
                               for trigSrcNameX in trigSrcNameList ]
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigSrc name err!", file=sys.stderr)
            self.statusBar().showMessage("trigSrc name err!", 1000)
            return

        global logTarNameList
        global logTarAddrList
        logTarNameList = []
        if (self.lineEdit_L.text() != ""):
            logTarNameList.append(self.lineEdit_L.text())
        if (self.lineEdit_L_2.text() != ""):
            logTarNameList.append(self.lineEdit_L_2.text())
        if (self.lineEdit_L_3.text() != ""):
            logTarNameList.append(self.lineEdit_L_3.text())
        if (self.lineEdit_L_4.text() != ""):
            logTarNameList.append(self.lineEdit_L_4.text())
        if (self.lineEdit_L_5.text() != ""):
            logTarNameList.append(self.lineEdit_L_5.text())
        if (self.lineEdit_L_6.text() != ""):
            logTarNameList.append(self.lineEdit_L_6.text())
        if (self.lineEdit_L_7.text() != ""):
            logTarNameList.append(self.lineEdit_L_7.text())
        if (self.lineEdit_L_8.text() != ""):
            logTarNameList.append(self.lineEdit_L_8.text())
        if (self.lineEdit_L_9.text() != ""):
            logTarNameList.append(self.lineEdit_L_9.text())
        if (self.lineEdit_L_10.text() != ""):
            logTarNameList.append(self.lineEdit_L_10.text())
        if (self.lineEdit_L_11.text() != ""):
            logTarNameList.append(self.lineEdit_L_11.text())
        if (self.lineEdit_L_12.text() != ""):
            logTarNameList.append(self.lineEdit_L_12.text())
        if (self.lineEdit_L_13.text() != ""):
            logTarNameList.append(self.lineEdit_L_13.text())
        if (self.lineEdit_L_14.text() != ""):
            logTarNameList.append(self.lineEdit_L_14.text())
        if (self.lineEdit_L_15.text() != ""):
            logTarNameList.append(self.lineEdit_L_15.text())
        if (self.lineEdit_L_16.text() != ""):
            logTarNameList.append(self.lineEdit_L_16.text())

        try:
            logTarAddrList = [varDict[logTarNameX]
                              for logTarNameX in logTarNameList]
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" logSrc name err!", file=sys.stderr)
            self.statusBar().showMessage("logSrc name err!", 1000)
            return

        global logTarCnt
        logTarCnt = len(logTarNameList)
        if (logTarCnt < 1):
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" logSrc not enough!", file=sys.stderr)
            self.statusBar().showMessage("logSrc not enough!", 1000)
            return

        # bufferSize
        global buffSize
        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_buffSize.text(), None, loc)
            buffSize = int(loc['val'])
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" buffSize err!", file=sys.stderr)
            self.statusBar().showMessage("buffSize err!", 1000)
            return

        # 最大采样tick
        global maxRecTick
        maxRecTick = int(buffSize/logTarCnt)

        global logKeepRatio
        global logKeepDataNumBeforeTrig
        logKeepRatio = self.doubleSpinBox.value()
        logKeepDataNumBeforeTrig = int(logKeepRatio * maxRecTick)*logTarCnt

        # 触发时间阈值
        global trigTimeTar
        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_trigtime.text(), None, loc)
            trigTimeTar = loc['val']
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigTimeTar err!", file=sys.stderr)
            self.statusBar().showMessage("trigTimeTar err!", 1000)
            return

        # 触发阈值
        global trigThd
        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_trigThd.text(), None, loc)
            trigThd = loc['val']
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigThd err!", file=sys.stderr)
            self.statusBar().showMessage("trigThd err!", 1000)
            return

        # 源模式
        # bit 0-2 类型: 0 int16; 1 uint16; 2 int32; 3 uint32; 4 float; 5 强制0;
        # bit 3 是否取反
        # src[0]在低位 src[1]在高位
        global trigSrc
        temp1 = self.comboBox.currentIndex()
        if (self.checkBox.checkState()):
            temp1 += 8
        temp2 = self.comboBox_2.currentIndex()
        if (self.checkBox_2.checkState()):
            temp2 += 8
        trigSrc = temp1+temp2*16

        # 间隔采样
        global sampInter
        sampInter = self.spinBox.value()
        # 触发模式
        global trigMode
        trigMode = self.comboBox_trigMode.currentIndex()

        try:
            setTrigConf()
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigCfg err!", file=sys.stderr)
            self.statusBar().showMessage("trigCfg err!", 1000)
        else:
            self.statusBar().showMessage("trigCfg OK!", 1000)

    def paraSetRSlot(self, nameEdit, valEdit):
        try:
            ret = readMCUVarUtil(nameEdit.text())
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" paraSetR err!", file=sys.stderr)
            self.statusBar().showMessage("paraSetR err!", 1000)
        else:
            if (ret != None):
                valEdit.setText(str(ret))
                self.statusBar().showMessage("read ok", 1000)

    def paraSetWSlot(self, nameEdit, valEdit):
        try:
            val = 0
            loc = {"val": 0}
            exec("val = "+valEdit.text()+"\n", None, loc)
            val = loc['val']

        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" val err!", file=sys.stderr)
            self.statusBar().showMessage("val err!", 1000)
        else:
            try:
                ret = setMCUVarUtil(nameEdit.text(), val)
            except Exception:
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" paraSetW err!", file=sys.stderr)
                self.statusBar().showMessage("paraSetW err!", 1000)
            else:
                if (ret == True):
                    self.statusBar().showMessage("write ok", 1000)

    def __paraTabInitUtil(self, nameEdit, valEdit, Rbtn, Wbtn):
        Rbtn.clicked.connect(functools.partial(
            self.paraSetRSlot, nameEdit, valEdit))
        Wbtn.clicked.connect(functools.partial(
            self.paraSetWSlot, nameEdit, valEdit))
        nameEdit.setCompleter(self.completer)

    def wait4TrigSlot(self):
        global waitingFlg
        if (waitingFlg):
            # 正在等就不等了
            waitingFlg = False
            self.pushButton_np.setText("wait for trig")
            self.statusBar().showMessage("manually canceled!", 1000)
        else:
            waitingFlg = True
            trigSta = 255
            self.pushButton_np.setText("waiting...")

            while (waitingFlg):
                QtWidgets.QApplication.processEvents()
                time.sleep(0.2)

                try:
                    trigSta = readMCUVarUtil("cdb1.trigSta")
                    pass
                except Exception:
                    waitingFlg = False
                    self.pushButton_np.setText("wait for trig")
                    self.statusBar().showMessage("CAN comm err while wait!", 1000)
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" CAN comm err while wait!", file=sys.stderr)
                    return

                if (trigSta >= 5):
                    waitingFlg = False
                    self.pushButton_np.setText("wait for trig")
                    self.statusBar().showMessage("trig detected!", 1000)
                    self.tabWidget.setCurrentIndex(2)
                    return


# 读取变量的辅助函数

def readMCUVarUtil(namex):
    global ch

    if (namex in varTypeDict and namex in varDict):
        match varTypeDict[namex]:
            case "uint16_t" | "int16_t" | "uint32_t" | "int32_t" | "float":
                ch = canlib.openChannel(
                    channel=0,
                    flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
                    bitrate=cdbBitrate,
                )
                ch.setBusOutputControl(canlib.Driver.NORMAL)
                ch.busOn()

                try:
                    match varTypeDict[namex]:
                        case "uint16_t" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<I", 0)
                            txFrame = Frame(id_=6, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<H", frame.data[4:6])[0]

                        case "int16_t" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<I", 0)
                            txFrame = Frame(id_=6, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<h", frame.data[4:6])[0]

                        case "uint32_t" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<I", 2)
                            txFrame = Frame(id_=6, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<I", frame.data[4:8])[0]

                        case "int32_t" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<I", 2)
                            txFrame = Frame(id_=6, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<i", frame.data[4:8])[0]

                        case "float" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<I", 2)
                            txFrame = Frame(id_=6, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<f", frame.data[4:8])[0]

                except Exception:
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" can HW err!")
                    can_val = None
                finally:
                    try:
                        ch.busOff()
                        ch.close()
                    except :
                        print("ch close err...\n")

                return can_val

            case _:
                print("type not supported!")
    else:
        print("type or name not registered!")

# 设置变量的辅助函数


def setMCUVarUtil(namex, val):
    global ch
    comSta = True

    if (namex in varTypeDict):
        match varTypeDict[namex]:
            case "uint16_t" | "int16_t" | "uint32_t" | "int32_t" | "float":

                ch = canlib.openChannel(
                    channel=0,
                    flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
                    bitrate=cdbBitrate,
                )
                ch.setBusOutputControl(canlib.Driver.NORMAL)
                ch.busOn()

                try:
                    match varTypeDict[namex]:
                        case "uint16_t" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<H", val) + struct.pack("<H", 0)
                            txFrame = Frame(id_=3, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<H", frame.data[4:6])[0]
                                if (abs(val-can_val)/(abs(val)+1e-12) > 0.01):
                                    print(datetime.datetime.now().strftime(
                                        '[%H:%M:%S.%f]')+" CAN err!")
                                    comSta = False

                        case "int16_t" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<h", val) + struct.pack("<H", 0)
                            txFrame = Frame(id_=3, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<h", frame.data[4:6])[0]
                                if (abs(val-can_val)/(abs(val)+1e-12) > 0.01):
                                    print(datetime.datetime.now().strftime(
                                        '[%H:%M:%S.%f]')+" CAN err!")
                                    comSta = False

                        case "uint32_t" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<I", val)
                            txFrame = Frame(id_=4, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<I", frame.data[4:8])[0]
                                if (abs(val-can_val)/(abs(val)+1e-12) > 0.01):
                                    print(datetime.datetime.now().strftime(
                                        '[%H:%M:%S.%f]')+" CAN err!")
                                    comSta = False

                        case "uint32_t" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<i", val)
                            txFrame = Frame(id_=4, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<i", frame.data[4:8])[0]
                                if (abs(val-can_val)/(abs(val)+1e-12) > 0.01):
                                    print(datetime.datetime.now().strftime(
                                        '[%H:%M:%S.%f]')+" CAN err!")
                                    comSta = False

                        case "float" :
                            txFrameData = struct.pack(
                                "<I", varDict[namex]) + struct.pack("<f", val)
                            txFrame = Frame(id_=4, data=txFrameData, dlc=8)
                            ch.write(txFrame)
                            ch.writeSync(timeout=200)

                            frame = ch.read(timeout=200)
                            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                                print(datetime.datetime.now().strftime(
                                    '[%H:%M:%S.%f]')+" frame err!")
                            else:
                                can_val = struct.unpack("<f", frame.data[4:8])[0]
                                if (abs(val-can_val)/(abs(val)+1e-12) > 0.01):
                                    print(datetime.datetime.now().strftime(
                                        '[%H:%M:%S.%f]')+" CAN err!")
                                    comSta = False

                except canlib.CanNoMsg:
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" no reply!")
                    comSta = False
                finally:
                    try:
                        ch.busOff()
                        ch.close()
                    except :
                        print("ch close err...\n")

                return comSta

            case _:
                print("type not supported!")
    else:
        print("type or name not registered!")

# 设置触发的辅助函数


def setTrigConf():
    # 通过can写入触发配置
    global ch
    global trigSrcNameList
    global trigSrcAddrList
    global logTarNameList
    global logTarAddrList
    global logTarCnt
    global buffSize
    global maxRecTick
    global logKeepDataNumBeforeTrig
    global trigTimeTar
    global trigThd
    global trigSrc
    global sampInter
    global trigMode

    ch = canlib.openChannel(
        channel=0,
        flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
        bitrate=cdbBitrate,
    )
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    try:
        # 源地址
        for ii in range(len(trigSrcAddrList)):
            txFrameData = struct.pack("<I", ii) + \
                struct.pack("<I", trigSrcAddrList[ii])
            txFrame = Frame(id_=2, data=txFrameData, dlc=8)
            ch.write(txFrame)
            QtWidgets.QApplication.processEvents()
            ch.writeSync(timeout=200)
            time.sleep(0.1)

        # 记录项
        for ii in range(len(logTarAddrList)):
            txFrameData = struct.pack("<I", ii + 2) + \
                struct.pack("<I", logTarAddrList[ii])
            txFrame = Frame(id_=2, data=txFrameData, dlc=8)
            ch.write(txFrame)
            QtWidgets.QApplication.processEvents()
            ch.writeSync(timeout=200)
            time.sleep(0.1)

        # 需记录的数据个数 18
        txFrameData = struct.pack("<I", 18) + struct.pack("<I", logTarCnt)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 需保存的触发前的数据个数 19
        txFrameData = struct.pack("<I", 19) + \
            struct.pack("<I", logKeepDataNumBeforeTrig)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 和时间有关触发的目标值 20
        txFrameData = struct.pack("<I", 20) + struct.pack("<I", trigTimeTar)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 触发阈值 21
        txFrameData = struct.pack("<I", 21) + struct.pack("<f", trigThd)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 触发值选择 22
        txFrameData = struct.pack("<I", 22) + struct.pack("<I", trigSrc)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 采样间隔 23
        txFrameData = struct.pack("<I", 23) + struct.pack("<I", sampInter)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 触发模式 24
        txFrameData = struct.pack("<I", 24) + struct.pack("<I", trigMode)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)

        # 触发器状态 25
        txFrameData = struct.pack("<I", 25) + struct.pack("<I", 1)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)

    finally:
        try:
            ch.busOff()
            ch.close()
        except :
            print("ch close err...\n")

# dump RAM的辅助函数


def dumpBuff():
    global ch
    ch = canlib.openChannel(
        channel=0,
        flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
        bitrate=cdbBitrate,
    )
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    global startAddr
    global endAddr
    txFrameData = struct.pack("<I", startAddr) + struct.pack("<I", endAddr)
    txFrame = Frame(id_=1, data=txFrameData, dlc=8)

    dumpErr=False

    try:
        ch.write(txFrame)
    except Exception:
        try:
            ch.busOff()
            ch.close()
        except :
            pass
        self.statusBar().showMessage("dump com fail!", 1000)
        print(datetime.datetime.now().strftime(
            '[%H:%M:%S.%f]')+" dump com fail!", file=sys.stderr)
        dumpErr = True
        return dumpErr

    global recvDict

    # 监听端口
    timeout = 0.2
    ticktime = 2
    tick_countup = 0
    frame_cnt = 0
    frame_cnt2 = 0
    frame_err = 0

    while True:
        try:
            frame = ch.read(timeout=int(timeout * 1000))
            # 最基本的校验
            if (frame.flags != 2 or frame.id < 8 or frame.id > 15 or frame.dlc != 8):
                frame_err += 1
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+"total CAN frame err cnt: {}\n".format(frame_err), file=sys.stderr)
                w.statusBar().showMessage("err cnt: {}".format(frame_err), 1000)
                continue

            frame_cnt += 1
            frame_cnt2 += 1
            if (frame_cnt2 == 100):
                w.statusBar().showMessage(f"recv {frame_cnt:>6} / all {dumpSize:>6} ({frame_cnt/dumpSize:>.2%})", 1000)
                QtWidgets.QApplication.processEvents()
                frame_cnt2 = 0

            can_addr = struct.unpack("<I", frame.data[0:4])[0]

            # raw
            can_val = frame.data[4:8]
            recvDict[can_addr-startAddr] = can_val

        except canlib.CanNoMsg:
            tick_countup += timeout

            if (frame_cnt == dumpSize):
                w.statusBar().showMessage("dump recv ok! cnt: {}".format(frame_cnt), 1000)
                QtWidgets.QApplication.processEvents()
                break

            if (frame_cnt > dumpSize):
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" recved {} pkgs, too more!".format(frame_cnt), file=sys.stderr)
                w.statusBar().showMessage("recved {} pkgs, too more!".format(frame_cnt), 1000)
                QtWidgets.QApplication.processEvents()
                dumpErr = True
                break

            if tick_countup > ticktime:
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" dump time out!", file=sys.stderr)
                w.statusBar().showMessage("dump time out!", 1000)
                QtWidgets.QApplication.processEvents()
                dumpErr = True
                break

        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" other dump err!", file=sys.stderr)
            w.statusBar().showMessage("other dump err!", 1000)
            QtWidgets.QApplication.processEvents()
            dumpErr = True
            break

    try:
        ch.busOff()
        ch.close()
    except :
        pass

    return dumpErr


def gui_task():
    global w
    global app
    app = QtWidgets.QApplication(sys.argv)
    fontx = QtGui.QFont()
    fontx.setPixelSize(14)
    app.setFont(fontx)

    w = mainWindow()
    w.show()

    sys.exit(app.exec())

############## main ##############


# 尝试打开端口
# 保证变量全局性？
ch = canlib.openChannel(
    channel=0,
    flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
    bitrate=cdbBitrate,
)
ch.setBusOutputControl(canlib.Driver.NORMAL)
ch.busOn()
ch.busOff()
ch.close()

# 建立触发配置的数据结构
trigSrcNameList = ["a"]
trigSrcAddrList = []
logTarNameList = ["a"]
logTarAddrList = []
logTarCnt = 1
buffSize = int(15 * 4 * 1024 / 2)
maxRecTick = int(buffSize/logTarCnt)
logKeepRatio = 0.2
logKeepDataNumBeforeTrig = int(logKeepRatio * maxRecTick)*logTarCnt
trigTimeTar = 40 * 2
trigThd = 280
trigSrc = 0b0101_0100
sampInter = 0
trigMode = 0

# 建立dump的数据结构
startAddr = 0x9000
dumpSize = buffSize
endAddr = startAddr+2*dumpSize
totalCycle = maxRecTick
cbStr = "Str 2 cb test"
recvDict = {}
valYnpTab = []
try:
    loc = {"val": 0}
    exec("val = " + IsrFreq_init, None, loc)
    IsrFreq = loc['val']
except Exception:
    print("IsrFreq_init err!")
timex = np.arange(totalCycle)/IsrFreq

# 等待标记
waitingFlg = False

# 窗口 workaround
w = 0
app = 0

gui_thread = threading.Thread(target=gui_task)
gui_thread.start()

# 提示字符
print("\n"+datetime.datetime.now().strftime('[%H:%M:%S.%f] ')
    + VERSION_STR+" started!\n")
