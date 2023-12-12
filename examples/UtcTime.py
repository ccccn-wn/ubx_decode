'''实时计算utc时间'''
from datetime import datetime
from datetime import timedelta
from datetime import timezone 
from time import time
import time
import pandas as pd

import ntplib
import requests
SHA_TZ = timezone(
    timedelta(hours=8),
    name='Asia/Shanghai',
)
def get_utcTime():
    utc_now = datetime.utcnow().replace(tzinfo=timezone.utc)
    return str(utc_now.date() ) + '-' + str(utc_now.hour) + '-' + str(utc_now.minute) + '-' + str(utc_now.second) 

def get_beijin_time():
    try:
        url = 'https://beijing-time.org/'
        request_result = requests.get(url=url)
        if request_result.status_code == 200:
            headers = request_result.headers
            net_date = headers.get("date")
            gmt_time = time.strptime(net_date[5:25], "%d %b %Y %H:%M:%S")
            bj_timestamp = int(time.mktime(gmt_time) + 8 * 60 * 60)
            return datetime.fromtimestamp(bj_timestamp)
    except Exception as exc:
        return datetime.now()

a = [1,2,3]
LogTime = str(get_utcTime())
df = pd.DataFrame({'GPSWeek': a})
df.to_excel(LogTime + '.xlsx', sheet_name='sheet1', index=False)
