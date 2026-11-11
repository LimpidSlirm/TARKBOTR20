# Camera：摄像头道路边界、中心线与曲率预览

支持照片、录制视频、显式指定摄像头；分别提取多种HSV颜色候选，配对当前车道左右边界，拟合局部中心线。标定有效时输出米制局部曲率、半径和偏差。提供短虚线候选提示和几何预瞄值，不连接串口/CAN或电机，不识别交通标志、不选择路口分支。

这是一份可运行的桌面Python/OpenCV初版。已有用户赛道屏幕录像用于颜色分割回归，几何/曲率测试仍来自合成像素图、已知解析几何及生成的视频，不能当作赛道识别准确率或实车验证。

合成演示示例见 `examples/合成S弯预览.png`，验证范围见 `examples/合成验证摘要.json`。

本目录由原视觉巡线模块整理而来，保留原目录供旧命令和历史记录使用。Camera可以独立放到其他项目中；依赖、运行记录和临时编译目录不属于源码交付。

## 目录与协作

- `main.py`、`run.ps1`：照片、视频、USB摄像头预览入口。
- `lane_vision/`：边线分割、片段与双边界配对、中心线、曲率及画面显示。
- `calibrate_ground.py`：地面标定工具。
- `config.track.json`：当前黄/红边界赛道配置；`config.example.json`：通用示例。
- `tests/`、`examples/`：31项测试、回归样本、演示和已记录的回放结果。
- `requirements.txt`、`requirements-tested.txt`：依赖范围与本机验证版本。
- `SOURCE_MANIFEST.json`：本次整理来源和逐文件SHA256，排除依赖、缓存和运行输出。

在其他电脑或独立仓库根目录运行（不需要Codex）：

```powershell
python -m venv Camera/.venv
.\Camera\.venv\Scripts\python.exe -m pip install -r Camera/requirements-tested.txt
.\Camera\.venv\Scripts\python.exe Camera/main.py --camera 1 --pixel-only --config Camera/config.track.json
.\Camera\.venv\Scripts\python.exe Camera/tests/run_tests.py
```

上传GitHub时，将Camera作为仓库根目录下的独立模块，与Robot并列；不要把Camera目录里的`.deps`、`.build`、`output`或虚拟环境一起上传。源码仍为Python/OpenCV，未移植为STM32的C/H程序，未增加电机通信。

## 本机立即运行

从 `C:\work\智能车` 打开PowerShell：

```powershell
# 生成直道、左右弯和S弯并显示窗口；Q或Esc退出
.\Camera\run.ps1 --demo

# 无窗口运行，保存PNG和JSON记录
.\Camera\run.ps1 --demo --no-display --max-frames 60

# 处理你的道路照片（默认只显示像素几何）
.\Camera\run.ps1 --image 'C:\实际位置\道路照片.jpg'

# 录制视频；不打开任何摄像头
.\Camera\run.ps1 --video 'C:\实际位置\道路视频.mp4'

# 只有主动指定这个选项才会打开摄像头，编号按本机实际设备选择
.\Camera\run.ps1 --camera 0

# 本次实际赛道：黄/红边线，排除白色反光（当前USB摄像头编号1）
.\Camera\run.ps1 --camera 1 --pixel-only --config .\Camera\config.track.json
```

本机已将OpenCV安装在本目录 `.deps/`，使用现有bundled Python与NumPy；未修改系统Python。`run.ps1`优先使用本目录虚拟环境，其次本机bundled Python，最后PATH中的Python。

若PowerShell不能运行脚本，可直接使用本机解释器：

```powershell
$visionPython = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
& $visionPython '.\Camera\main.py' --demo
```

其他电脑安装Python 3.11+，建议建立虚拟环境，再执行 `python -m pip install -r Camera/requirements.txt`，使用该解释器运行 `Camera/main.py`。本轮实测Python3.12.14、NumPy2.3.5、OpenCV4.14.0（wheel 4.14.0.94）。依赖目录、输出和虚拟环境不纳入Git。

## 当前处理流程与适用范围

1. 在ROI内分别用配置的HSV范围提取候选，合并后做3×3开运算去散点，不做跨大片缺口的闭运算。
2. 从近处开始逐行扫描，仅考虑相邻候选线之间的车道，初始车道须包含配置的前进参考线。按最大位移/宽度变化向远处关联；邻近竞争边界标记AMBIGUOUS，长缺口立即截断支持区间。
3. 左右边界分别用归一化自变量的低阶多项式及残差剔除拟合，保留共同有效范围。未标定仅显示像素中心和相对车道半宽归一化偏差。
4. 标定后分别把左右观测变换到X前Y左的米制坐标，拟合Y_left(X)、Y_right(X)，在共同区间构造中点并拟合中心线。输出局部曲率 `Y''/(1+Y'^2)^(3/2)`，正为左弯、负为右弯；近似直线半径以null加radius_is_infinite=true表示。
5. 短缺口插值与实测分开标记，插值点不产生有效曲率；多次短缺口只标记dashed_candidate，不把虚线自动认定为交叉口。长缺口只保留近处支持，不能向远处猜路。

初版是单帧保守关联，没有跨帧边界跟踪、完整样条/法向中点、复杂透视下的自适应窗口或完整交叉口分类。固定像素门限依赖分辨率；相机初始朝向须在当前车道内，路口/急弯/遮挡可能被拒绝，需要真实照片调参。对大航向角或高度弯曲的道路，同X中点近似会产生误差；本版通过斜率和宽度门限拒绝明显不适用结果，不能保证覆盖所有几何情况。

## 调整你的地图参数

复制 `config.example.json` 为新配置，通过 `--config '路径.json'` 加载。默认白/黄HSV只是合成演示与初调起点，不代表实际地图颜色规则。

- `colors`：每个颜色具有唯一name、lower/upper三元素整数范围；H为0～179，S/V为0～255。红色等跨Hue端点时写成两个不同名称的范围。
- `roi_top/roi_bottom`：道路图像纵向范围比例；`reference_x`是当前车道的前进参考横位置比例，默认0.5须实测校正。
- `min/max_lane_width_fraction`、`min/max_run_px`：行扫描的车道宽度与标线粗细门限。
- `scan_step`、`max_step_shift_px`：逐行关联步长与位移门限；`max_gap_rows`控制允许跨越的短缺口，不是路口通行许可。
- `min_pairs`、`min_coverage`、`degree`、`pixel/metric_rmse_limit`：有效支持与拟合质量门限。
- `min/max_lane_width_m`、`min_metric_span_m`、`max_heading_slope`：米制几何有效区域。
- `lookahead_x_m`：只用于显示几何κ_track，只有该位置位于可靠观测支持内才输出；当前没有速度请求、百分比换算或运动发送。

## 本次赛道的反光调试

`config.track.json`是独立的黄/红边线配置；原白/黄通用配置和合成演示保持原有用途。录像里浅黄色标线的饱和度低于原黄色下限75，部分明亮蓝灰地面则落入白色范围。赛道配置将黄色下限改为S=28、V=65，H=15～45；红色分成两个Hue范围。白色箭头、斑马线和白色高光不参与边界配对。若未来确有白色道路边界，需要另行启用并加入形状/位置约束，不能直接套此配置。

ROI顶端设为0.4以减少远处背景，标线最大扫描宽度64像素适配本次640×480的较近粗线；这些是调试起点，不是相机标定。画面顶部显示Boundary bands，右侧显示经过ROI裁剪和去噪、真正参与配对的候选区域，不再显示被忽略的天花板/背景原始颜色掩码。

本次录像是1920×900屏幕录制，其中原相机画面为左侧(0,120,640,480)；回放分析仅裁出该区域，未将右侧旧掩码或黑边作为相机输入。tests/fixtures/lighting-road.png是第0帧的纯地面局部裁剪，来源和裁剪记录见同名JSON，只用于分割回归，不包含车道/米制真值。

本次修改解决颜色误检：不放宽分支歧义、近处种子、丢线或标定门限。录像中另有一侧边线出画、参考中线落到路外、抬头及倾斜，仍可能LOST/AMBIGUOUS。请把相机固定在车头中间，朝前下方，使画面中下部能同时看到当前车道两侧边界。完全过曝成白色的黄色线没有可靠颜色信息，本版会拒绝，不凭空补线；此时需降低曝光或改善照明/反光，并重新验证。当前未修改相机硬件曝光、增益或白平衡。

从任何PowerShell目录启动：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\work\智能车\Camera\run.ps1" --camera 1 --pixel-only --config "C:\work\智能车\Camera\config.track.json"
```

相关测试覆盖真实局部浅黄保留/明亮非边线抑制、亮度变化、红色Hue跨零、完全白色拒绝、ROI/去噪预览一致性。它们不能证明本赛道完整双边界跟踪或实车闭环通过。

## 真实相机标定

无 `--calibration` 时，metric_valid=false、center_points为空、tracking_preview=null；绝不会把像素曲线当作米制曲率。分辨率与标定不一致时撤销物理量有效性，保留像素预览。

1. 用实际相机完成内参/镜头畸变标定，固定安装、焦距、分辨率和裁剪。
2. 在去畸变后的图像上选至少4个非退化地面控制点，尺量其X前Y左坐标，原点须是车体控制参考点。另选至少2个不参与求解的独立验证点。
3. 按 `examples/ground_points.synthetic.json` 的字段建立你自己的点文件，改为真实calibration_id、图像尺寸与坐标，真实数据设置synthetic=false。该示例全部是人工生成的已知几何，不能拿它标定真实相机。
4. 若传入原始畸变图像，提供camera_matrix及distortion_coefficients，程序先用它们去畸变；控制点始终应在以同一K去畸变后的图像上测量。若上游已去畸变，camera_matrix=null并明确input_is_undistorted=true。
5. 执行下方工具。它会验证点不共线、矩阵可逆、验证点独立、误差不超过指定门限；不会仅信任一个validated标志。标定文件不会覆盖同名旧文件。

```powershell
python Camera/calibrate_ground.py --points '你的地面点.json' --output '你的标定-v1.json'
python Camera/main.py --image '道路照片.jpg' --calibration '你的标定-v1.json'
```

工具只求解地面单应矩阵，不代替相机内参采集或实际尺量。分辨率一致不能证明安装仍未变化；需由操作者管理标定版本、安装和重新验证。远离控制点覆盖区域、坡面和相机振动的误差仍需实测。

## 输出与画面

默认写入 `output/日期时间/`；`--output`可指定新目录，目录非空时拒绝覆盖。`--save-every N`设置PNG间隔，结束另存last_frame.png。

- 左侧：橙色描边显示实际颜色片段（未指定为左右边界），蓝/黄点显示已配对扫描观测；只有中心线可用时绘制绿色中心线。紫色点表示非直接观测插值。
- NEAR/MID/FAR固定为ROI的下/中/上三分区，显示配对行数和区域偏差；观测覆盖不足或全局配对无效时显示CENTER N/A。它们是图像区域，不是米制距离，不再把短拟合曲线重新均分为“近中远”。
- 右侧：颜色候选及有效曲率随前向距离的有符号图。
- JSONL：逐帧状态、失效原因、支持区间、像素样本、米制中心点/曲率/半径、逐点valid/observed、标定ID、处理时间和观察到的预览帧率。
- 诊断字段：boundary_fragments为ROI/去噪后的颜色片段轮廓，最多保留128个，boundary_fragment_count及fragments_truncated说明截断；paired_observations即使不足以拟合也保留。pairing_diagnostics区分near_seed_missing、long_gap、competing_boundaries、competing_pairs和roi_end。centerline_available控制有效绿色路径显示；relative_image_zones报告固定图像分区的配对行数、覆盖和偏差，区域覆盖不足时偏差为null。它们不能作为交叉口选路/运动许可。
- `capture_time_ns`为本机读取完成的单调时钟估计，不是硬件曝光时间；视频另存媒体位置。处理耗时和预览帧率不等于摄像头到控制端的完整延迟。
- `run.json`记录配置、源类型、依赖版本和帧数。所有记录command_sent=false，本版没有通信/电机控制模块。

`--demo`使用专用合成标定，画面标注SYNTHETIC，记录source_kind=synthetic和synthetic_calibration=true；可加 `--pixel-only`演示未标定模式。真实图像从不自动使用演示标定。

## 测试

```powershell
$visionPython = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
& $visionPython '.\Camera\tests\run_tests.py'
```

覆盖标定/独立验证、畸变前提、坐标方向、直道、偏离中线、左右曲率/半径、S弯、丢线、相邻车道干扰、分支歧义、短虚线/长缺口、分辨率变化、中文照片路径、录制视频和无窗口输出。几何与CLI检查使用生成图像/视频；自动化测试不打开摄像头或串口。

另含实际地面裁剪的反光回归，以及片段/配对/中心线分层显示、固定图像分区、无效拟合不画绿色路径等检查。当前共31项。对20-47-29虚线录像的466帧回放全部显示near_seed_missing，虽然每帧均有颜色片段，但无已配对扫描行；新诊断不会将这些片段伪装成完整道路。与修改前逐帧核对，原配对、像素几何、米制几何及预瞄字段完全一致。仍需固定相机朝向、真实标定及独立边界/跨帧/路口模型才能继续完善；详见examples/虚线片段回放摘要.json。

方案与阶段性验收仍见上级工作区 `测试准备/docs/视觉巡线与差速转向初步方案.md` 和对应验收用例；其中全部真实图像/实车项仍待数据和阶段验证，不能把合成测试批量标为通过。

参考：[OpenCV颜色范围分割](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html)、[相机标定与几何](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)、[图像解码/编码](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html)。
