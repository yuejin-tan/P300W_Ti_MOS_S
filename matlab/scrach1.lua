scd_tabItemClearAll()
scd_tabItemSelect("CH1_Ifilt.d")
scd_tabItemSelect("CH1_Ifilt.q")
scd_tabItemSelect("targetThetaE_CH1")
scd_tabItemSelect("thetaEnco_raw")
scd_tabItemSelect("targetIq_CH1")
scd_tabItemSelect("CH1_Udc")

scd_plotCfg()

scd_deltaPkgSet(-1)

scd_plotItemClearAll()
scd_plotItemSelect("CH1_Ifilt.d")
scd_plotItemSelect("CH1_Ifilt.q")
scd_plotItemSelect("targetThetaE_CH1")
scd_plotItemSelect("thetaEnco_raw")
scd_plotItemSelect("targetIq_CH1")
scd_plotItemSelect("CH1_Udc")
scd_YaxisSet("CH1_Ifilt.d", -19.9791, 10.5848)
scd_YaxisSet("CH1_Ifilt.q", -24.1203, 6.40261)
scd_YaxisSet("targetThetaE_CH1", -0.930443, 3.38703)
scd_YaxisSet("thetaEnco_raw", -23154.8, 432201)
scd_YaxisSet("targetIq_CH1", -25.0815, 6.11222)
scd_YaxisSet("CH1_Udc", -43.0881, 27.2147)

-- 241222
scd_tabItemClearAll()
scd_tabItemSelect("thetaEnco_raw")
scd_tabItemSelect("thetaEnco_raw2")
scd_tabItemSelect("CH1_Ifilt.d")
scd_tabItemSelect("CH1_Ifilt.q")
scd_tabItemSelect("CH2_Ifilt.d")
scd_tabItemSelect("CH2_Ifilt.q")
scd_tabItemSelect("CH1_Udc")

scd_plotCfg()

scd_deltaPkgSet(-1)

scd_plotItemClearAll()
scd_plotItemSelect("thetaEnco_raw")
scd_plotItemSelect("thetaEnco_raw2")
scd_plotItemSelect("CH1_Ifilt.d")
scd_plotItemSelect("CH1_Ifilt.q")
scd_plotItemSelect("CH2_Ifilt.d")
scd_plotItemSelect("CH2_Ifilt.q")
scd_plotItemSelect("CH1_Udc")
scd_YaxisSet("thetaEnco_raw", -24682.4, 418401)
scd_YaxisSet("thetaEnco_raw2", -25336.4, 430019)
scd_YaxisSet("CH1_Ifilt.d", -19.6618, 11.878)
scd_YaxisSet("CH1_Ifilt.q", -21.5477, 10.223)
scd_YaxisSet("CH2_Ifilt.d", -9.88879, 20.6941)
scd_YaxisSet("CH2_Ifilt.q", -9.26571, 21.3513)
scd_YaxisSet("CH1_Udc", -81.5762, 32.9002)

