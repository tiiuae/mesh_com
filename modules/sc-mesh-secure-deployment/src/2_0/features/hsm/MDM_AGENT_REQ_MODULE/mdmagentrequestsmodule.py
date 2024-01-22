from m2requests import M2HttpsAdapter
from requests import Session
import pprint, json
request = Session()
request.mount("https://", M2HttpsAdapter())
M2HttpsAdapter.check_hostname = False
r = request.get("https://127.0.0.1:4443/",verify='./rootCA.crt',cert=('pkcs11:token=rsa','pkcs11:token=rsa'))
print(r.status_code)
#pprint.pprint(json.loads(r.raw.data.decode('utf-8')))
