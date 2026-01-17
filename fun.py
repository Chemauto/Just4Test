import time
from tqdm import tqdm

# 发呆10s
def action():
    time.sleep(0.1)
with tqdm(total=100, desc='Example', leave=True, ncols=100, unit='B', unit_scale=True) as pbar:
    for i in range(100):
        # 发呆10秒
        action()
        # 更新发呆进度
        pbar.update(1)