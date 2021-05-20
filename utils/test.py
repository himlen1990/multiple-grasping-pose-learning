import json
from collections import OrderedDict
a = OrderedDict([
    ("frame00.jpg","1234"),
    ("asd","2234")
    ])
with open("test.json","w") as f:
    json.dump(a,f,sort_keys=False)
