```python
  "RXM-RAWX": {

​    "rcvTow": R8,

​    "week": U2,

​    "leapS": I1,

​    "numMeas": U1,

​    "recStat": (

​      X1,

​      {

​        "leapSec": U1,

​        "clkReset": U1,

​      },

​    ),

​    "reserved1": U3,

​    "group": (

​      "numMeas",

​      {  # repeating group * numMeas

​        "prMes": R8,

​        "cpMes": R8,

​        "doMes": R4,

​        "gnssId": U1,

​        "svId": U1,

​        "sigId": U1,

​        "freqId": U1,

​        "locktime": U2,

​        "cno": U1,

​        "prStdev": (

​          X1,  # scaling = 0.01*2^-n

​          {

​            "prStd": U4,

​          },

​        ),

​        "cpStdev": (

​          X1,  # scaling = 0.004

​          {

​            "cpStd": U4,

​          },

​        ),

​        "doStdev": (

​          X1,  # scaling = 0.002*2^n

​          {

​            "doStd": U4,

​          },

​        ),

​        "trkStat": (

​          X1,

​          {

​            "prValid": U1,

​            "cpValid": U1,

​            "halfCyc": U1,

​            "subHalfCyc": U1,

​          },

​        ),

​        "reserved3": U1,

​      },

​    ),

  },
```

