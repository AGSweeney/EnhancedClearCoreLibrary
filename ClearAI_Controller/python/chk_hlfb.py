import json
from clearai.client import ClearAiClient, discover

c = ClearAiClient()
c.connect_tcp(discover() or "172.16.82.113")
st = c.call("get_status")
print(json.dumps({k: st.get(k) for k in
    ("enabled", "hlfb", "moving", "alert_reg", "alerts_decoded", "alert_reg_axis", "hlfb_percent")}, indent=2))
c.close()
