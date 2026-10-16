# -*- coding: utf-8 -*-
"""用桩模块在无 ROS 环境下验证「换车自适应」逻辑（resolve_model_path / resolve_camera_topic）"""
import os
import shutil
import sys
import tempfile
import types

# ---------------- 桩掉 rospy 与 ROS 消息类型 ----------------
rospy = types.ModuleType('rospy')
LOGS = []
rospy.loginfo = lambda *a, **k: LOGS.append(('INFO', a[0] % a[1:] if len(a) > 1 else a[0]))
rospy.logwarn = lambda *a, **k: LOGS.append(('WARN', a[0] % a[1:] if len(a) > 1 else a[0]))
rospy.logerr = lambda *a, **k: LOGS.append(('ERR', a[0] % a[1:] if len(a) > 1 else a[0]))
rospy.Publisher = rospy.Subscriber = rospy.Rate = lambda *a, **k: None


class _M(object):
    def getPublishedTopics(self, sub=''):
        return [[t, 'x'] for t in sorted(PUBLISHED)]


rospy.get_master = lambda: _M()
sys.modules['rospy'] = rospy

for pkg in ('abot_driver', 'geometry_msgs', 'nav_msgs'):
    sys.modules.setdefault(pkg, types.ModuleType(pkg))
for mod, names in (('abot_driver.msg', ['Akm']),
                   ('geometry_msgs.msg', ['Twist']),
                   ('nav_msgs.msg', ['Odometry'])):
    m = types.ModuleType(mod)
    for n in names:
        setattr(m, n, type(n, (), {}))
    sys.modules[mod] = m

PUBLISHED = set()
SCRIPT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))
import race_abot_411_common as C  # noqa: E402

OK_ALL = True


def chk(cond, msg):
    global OK_ALL
    print(('  PASS  ' if cond else '  FAIL  ') + msg)
    OK_ALL = OK_ALL and bool(cond)


def hr(title):
    print('\n' + '=' * 62)
    print(title)
    print('=' * 62)


tmp = tempfile.mkdtemp()
try:
    models = os.path.join(tmp, 'catkin_ws', 'src', 'traffic_detector', 'models')
    os.makedirs(models)
    open(os.path.join(models, 'best.pt'), 'w').write('x')
    open(os.path.join(models, 'aaabest.pt'), 'w').write('x')

    hr('1) resolve_model_path：候选列表命中，aaabest 优先')
    p, _ = C.resolve_model_path('', candidates=[
        os.path.join(models, 'aaabest.pt'), os.path.join(models, 'best.pt')])
    chk(p.endswith('aaabest.pt'), '选中 %s' % os.path.basename(p or '(空)'))

    hr('2) 配置了不存在的路径 -> 自动回退到候选项 + WARN')
    LOGS[:] = []
    p2, _ = C.resolve_model_path('/home/abot/yolov8/best.pt',
                                 candidates=[os.path.join(models, 'aaabest.pt')])
    chk(p2.endswith('aaabest.pt'), '回退到 %s' % os.path.basename(p2 or '(空)'))
    warns = [m for lv, m in LOGS if lv == 'WARN']
    chk(bool(warns), '打出 WARN：%s' % (warns[-1] if warns else '(无)'))

    hr('3) 候选项都不存在 -> glob 兜底，aaabest 优先')
    d2 = os.path.join(tmp, 'other')
    os.makedirs(d2)
    open(os.path.join(d2, 'best.pt'), 'w').write('x')
    open(os.path.join(d2, 'aaabest.pt'), 'w').write('x')
    p3, _ = C.resolve_model_path('/nonexistent/x.pt',
                                 candidates=['/nonexistent/a.pt'],
                                 globs=[os.path.join(d2, '*.pt')])
    chk(p3.endswith('aaabest.pt'), 'glob 找到 %s' % os.path.basename(p3 or '(空)'))

    hr('4) 全都找不到 -> 返回不可用路径，并记录检查过的路径')
    p4, tried4 = C.resolve_model_path('/nonexistent/x.pt',
                                      candidates=['/nonexistent/a.pt'],
                                      globs=['/nonexistent/**/*.pt'])
    chk(not os.path.isfile(p4), '返回路径不可用（节点会打 ERR 并列出已检查项）')
    chk(len(tried4) >= 2, '记录了 %d 条检查过的路径' % len(tried4))

    hr('5) resolve_camera_topic：自动挑真在发布的话题')
    PUBLISHED.clear()
    PUBLISHED.update(['/camera/rgb/image_raw', '/camera/back/image_raw'])
    t, _ = C.resolve_camera_topic('', wait_s=1.5, name='前视相机')
    chk(t == '/camera/rgb/image_raw', '自动选中 %s' % t)

    hr('6) 配置的相机话题没人发布 -> 自动回退 + WARN')
    LOGS[:] = []
    t2, _ = C.resolve_camera_topic('/camera/right/image_raw', wait_s=1.5, name='标志相机')
    chk(t2 == '/camera/rgb/image_raw', '回退到 %s' % t2)
    warns = [m for lv, m in LOGS if lv == 'WARN']
    chk(bool(warns), '打出 WARN：%s' % (warns[-1] if warns else '(无)'))

    hr('7) 配置的相机话题确实存在 -> 尊重配置')
    PUBLISHED.clear()
    PUBLISHED.update(['/camera/right/image_raw', '/camera/rgb/image_raw'])
    t3, _ = C.resolve_camera_topic('/camera/right/image_raw', wait_s=1.5)
    chk(t3 == '/camera/right/image_raw', '选中 %s' % t3)

    hr('8) 一个发布者都没有 -> 兜底到候选首位，不崩')
    PUBLISHED.clear()
    t4, _ = C.resolve_camera_topic('', wait_s=1.0)
    chk(t4 == '/camera/rgb/image_raw', '兜底到 %s' % t4)

    hr('9) yaml 里的候选配置能被正确读取')
    import yaml
    cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'config', 'race_abot_411_params.yaml')
    cfg = yaml.safe_load(open(cfg_path, encoding='utf-8'))
    d = cfg['detect']
    tp = cfg['topics']
    chk(d.get('model_path') == '', 'detect.model_path 留空（自动查找）')
    chk(len(d.get('model_candidates') or []) >= 4, 'model_candidates %d 条' % len(d.get('model_candidates') or []))
    chk(len(d.get('model_globs') or []) >= 3, 'model_globs %d 条' % len(d.get('model_globs') or []))
    chk(tp.get('camera_front') == '' and tp.get('camera_sign') == '', 'camera_front/sign 留空（自动）')
    chk(len(tp.get('camera_candidates') or []) >= 5, 'camera_candidates %d 条' % len(tp.get('camera_candidates') or []))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

hr('结果: ' + ('全部通过 ✓' if OK_ALL else '有失败 ✗'))
sys.exit(0 if OK_ALL else 1)
