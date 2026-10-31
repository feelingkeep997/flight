// 网页遥控器客户端界面

#if WEB_RC_ENABLED

const char webRCIndexHtml[] = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no">
<title>琛光无人机遥控器</title>
<style>
/*======== 响应式变量 ========*/
:root{
  --js-size:clamp(150px,min(50vw,62vh),420px);
  --knob-size:calc(var(--js-size)*0.25);
  --pad:clamp(6px,1.5vmin,15px);
  --gap:clamp(6px,1.5vmin,15px);
}
/*======== 通用样式 ========*/
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent;touch-action:none;user-select:none}
body{font-family:'Roboto Mono',Arial,"Microsoft YaHei",sans-serif;background:#3c3c3c;color:#fff;overflow:hidden;height:100vh;height:100dvh;width:100vw}
.container{width:100%;height:100%;display:flex;flex-direction:column;padding:var(--pad);gap:var(--gap);max-width:1200px;margin:0 auto;overflow:hidden}

/*======== 顶部状态栏 ========*/
.header {
  text-align: center;
  padding: 6px 10px;
  background: rgba(30,30,30,.9);
  border-radius: 12px;
  border: 2px solid rgba(150,150,150,.3);
  box-shadow: 0 4px 16px rgba(0,0,0,.4);
  backdrop-filter: blur(10px)
}

.header h1 {
  font-size: 1.1rem;
  color: #ffffff;
  margin-bottom: 4px;
}

.status-bar {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: clamp(4px,1.2vmin,10px);
  flex-wrap: nowrap;
  overflow: hidden;
  margin-top: 4px;
}

.status-item {
  display: flex;
  align-items: center;
  gap: 4px;
  padding: 2px 5px;
  background: rgba(0,0,0,.3);
  border-radius: 6px;
  border: 1px solid rgba(255,255,255,.1);
  font-size: clamp(0.58rem,1.8vmin,0.7rem);
  white-space: nowrap;
  flex-shrink: 1;
  min-width: 0;
}

.status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  display: inline-block;
}

.status-dot.connected { background: #0f8; box-shadow: 0 0 8px #0f8; }
.status-dot.disconnected { background: #f33; box-shadow: 0 0 8px #f33; }
.status-dot.warning { background: #ff9; box-shadow: 0 0 8px #ff9; }

/*======== 内容区 ========*/
.content{display:flex;flex:1;gap:var(--gap);overflow:hidden;min-height:0}
.joystick-container{flex:1;display:flex;flex-direction:column;justify-content:center;align-items:center;background:rgba(0,0,0,.4);border-radius:20px;padding:clamp(8px,1.5vmin,15px);border:2px solid rgba(255,255,255,0.15);box-shadow:inset 0 0 30px rgba(0,0,0,.5)}
.joystick-title{font-size:clamp(0.75rem,2.2vmin,1.1rem);color:#cccccc}
.joystick-wrapper{width:100%;height:var(--js-size);display:flex;justify-content:center;align-items:center;position:relative;margin-top:clamp(4px,2vh,20px);}
.joystick{width:var(--js-size);height:var(--js-size);background:radial-gradient(circle at 30% 30%,rgba(255,255,255,.1),rgba(0,0,0,.3));border-radius:50%;position:relative;border:2px solid rgba(255,255,255,0.2);box-shadow:inset 0 0 20px rgba(0,0,0,.5),0 8px 25px rgba(0,0,0,.5);overflow:hidden}
.joystick::before{content:'';position:absolute;top:50%;left:50%;width:2px;height:100%;background:linear-gradient(to bottom,transparent,rgba(255,255,255,.15),transparent);transform:translate(-50%,-50%)}
.joystick::after{content:'';position:absolute;top:50%;left:50%;width:100%;height:2px;background:linear-gradient(to right,transparent,rgba(255,255,255,.15),transparent);transform:translate(-50%,-50%)}
.joystick-knob{width:var(--knob-size);height:var(--knob-size);background:radial-gradient(circle at 30% 30%,#fff,#cccccc);border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);border:2px solid rgba(255,255,255,.7);box-shadow:0 4px 15px rgba(0,0,0,.5),inset 0 0 10px rgba(255,255,255,.5);cursor:move;transition:transform .1s ease-out;z-index:10}

/*======== 左手区：油门推拉杆（自锁）+ 偏航滑杆（回中）========*/
.stick-panel{width:100%;flex:1;min-height:0;display:flex;flex-direction:column;justify-content:center;align-items:center;gap:clamp(6px,1.6vmin,14px)}
.thr-row{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:clamp(4px,1vmin,8px);flex:1;min-height:0;width:100%}
.stick-cap{font-size:clamp(0.6rem,1.8vmin,0.78rem);color:#bbbbbb;letter-spacing:1px;flex-shrink:0}
.thr-track{position:relative;flex:1;min-height:90px;width:clamp(46px,10vmin,66px);background:linear-gradient(to right,rgba(0,0,0,.45),rgba(255,255,255,.06) 50%,rgba(0,0,0,.45));border:2px solid rgba(255,255,255,.18);border-radius:999px;box-shadow:inset 0 0 18px rgba(0,0,0,.6);cursor:grab}
.thr-fill{position:absolute;left:2px;right:2px;bottom:2px;height:0%;border-radius:999px;background:linear-gradient(to top,rgba(0,255,136,.22),rgba(0,255,136,.7));pointer-events:none}
.thr-notch{position:absolute;left:10%;right:10%;top:50%;height:2px;background:rgba(255,255,255,.16);pointer-events:none}
.thr-thumb{position:absolute;left:50%;top:100%;width:calc(100% - 12px);height:clamp(24px,5.4vmin,38px);transform:translate(-50%,-50%);border-radius:12px;background:linear-gradient(180deg,#f4f4f4,#9a9a9a);box-shadow:0 3px 10px rgba(0,0,0,.55),inset 0 1px 0 rgba(255,255,255,.7);pointer-events:none;z-index:5}
.yaw-row{display:flex;align-items:center;justify-content:center;gap:clamp(5px,1.4vmin,10px);width:100%;flex-shrink:0}
.yaw-track{position:relative;flex:1;min-width:64px;max-width:var(--js-size);height:clamp(36px,7vmin,50px);background:linear-gradient(180deg,rgba(0,0,0,.45),rgba(255,255,255,.06) 50%,rgba(0,0,0,.45));border:2px solid rgba(255,255,255,.18);border-radius:999px;box-shadow:inset 0 0 16px rgba(0,0,0,.6);cursor:grab}
.yaw-center{position:absolute;left:50%;top:16%;bottom:16%;width:2px;transform:translateX(-50%);background:rgba(255,255,255,.22);pointer-events:none}
.yaw-thumb{position:absolute;top:50%;left:50%;width:clamp(24px,5.2vmin,36px);height:calc(100% - 12px);transform:translate(-50%,-50%);border-radius:12px;background:linear-gradient(180deg,#f4f4f4,#9a9a9a);box-shadow:0 3px 10px rgba(0,0,0,.55),inset 0 1px 0 rgba(255,255,255,.7);pointer-events:none;z-index:5}
.stick-read{font-family:'Roboto Mono',Consolas,monospace;font-size:clamp(0.6rem,1.7vmin,0.76rem);color:#00ff88;min-width:3em;text-align:center;flex-shrink:0}
.thr-track.active,.yaw-track.active{border-color:rgba(0,255,136,.55);box-shadow:inset 0 0 18px rgba(0,0,0,.6),0 0 14px rgba(0,255,136,.3)}

/*======== 按钮区 ========*/
.buttons-container{flex:.55;display:flex;flex-direction:column;gap:10px;padding:12px;background:rgba(0,0,0,.4);border-radius:20px;border:2px solid rgba(150,150,150,.3);box-shadow:inset 0 0 20px rgba(0,0,0,.5)}
.buttons-grid{display:grid;grid-template-columns:repeat(6,1fr);grid-template-rows:repeat(2,1fr);gap:8px;flex:1}
.buttons-grid>button{grid-column:span 2}
.button{background:linear-gradient(145deg,#484848,#383838);border:none;border-radius:10px;color:#fff;font-size:.85rem;font-weight:bold;display:flex;flex-direction:column;justify-content:center;align-items:center;text-align:center;padding:10px 5px;cursor:pointer;transition:all .15s cubic-bezier(.4,0,.2,1);box-shadow:0 3px 10px rgba(0,0,0,.3),inset 0 1px 0 rgba(255,255,255,.1);position:relative}
.button:hover{background:linear-gradient(145deg,#565656,#464646);transform:translateY(-1px)}
.button.active{background:linear-gradient(145deg,#1a73e8,#0d47a1);box-shadow:0 0 15px rgba(26,115,232,.6),inset 0 1px 0 rgba(255,255,255,.2);transform:scale(.95)}
.button:active{transform:scale(.92)}
.button-icon{font-size:1.1rem;margin-bottom:4px}

/*======== 调试控制台 ========*/
.console-panel{background:rgba(10,10,10,.95);border-radius:12px;border:1px solid rgba(100,100,100,.4);padding:10px;flex-shrink:0;max-height:65vh;display:flex;flex-direction:column;gap:6px}
/* 输出区：允许触摸滚动（touch-action）、允许长按选中文字，高度加大便于手机查看/复制 */
.console-output{flex:1;overflow-y:auto;font-family:'Courier New',monospace;font-size:0.7rem;color:#00ff88;min-height:140px;max-height:45vh;word-break:break-all;-webkit-overflow-scrolling:touch;touch-action:pan-y;-webkit-user-select:text;user-select:text;overscroll-behavior:contain}
/* touch-action/user-select 都不继承：顶部 * 通配规则已把每行日志钉成 none，
   触摸滚动判定 = 手指所在行 ∩ 最近滚动容器(本面板 pan-y)，行是 none 则整块滚不动；
   必须对子元素显式放开，长按选择文字同理 */
.console-output div{padding:1px 0;border-bottom:1px solid rgba(255,255,255,.03);touch-action:pan-y;-webkit-user-select:text;user-select:text}

/*======== 动画 ========*/
@keyframes pulse{0%{box-shadow:0 0 0 0 rgba(67,97,238,.7)}70%{box-shadow:0 0 0 12px rgba(67,97,238,0)}100%{box-shadow:0 0 0 0 rgba(67,97,238,0)}}
.joystick.active{animation:pulse 1.5s infinite}
@keyframes fadeIn{from{opacity:0;transform:translateY(20px)}to{opacity:1;transform:translateY(0)}}
.container>*{animation:fadeIn .5s ease-out}

/*======== 竖屏自适应 ========*/
@media (orientation:portrait){
  :root{--js-size:clamp(120px,40vw,240px);--knob-size:calc(var(--js-size)*0.25)}
  .content{
    display:grid;
    grid-template-columns:repeat(2,minmax(0,1fr));
    grid-template-rows:auto auto;
    height:auto;
    flex:none;
    overflow:visible;
  }
  .buttons-container{grid-column:1/3;grid-row:1}
  #panel-left{grid-column:1;grid-row:2;min-width:0;overflow:hidden}
  #panel-right{grid-column:2;grid-row:2;min-width:0;overflow:hidden}
  /* 竖屏时网格行高是 auto，油门轨道会塌到最小值，这里显式给足行程 */
  #panel-left .thr-track{min-height:min(34vh,200px)}
  #panel-left .stick-panel{padding:clamp(2px,1vmin,8px) 0}
  .header h1{font-size:clamp(0.85rem,3.5vw,1.1rem)}
  .status-bar{gap:5px;flex-wrap:wrap;justify-content:center}
  .status-item{font-size:clamp(0.6rem,2.5vw,0.7rem);padding:2px 5px}
}

/*======== 小屏横屏自适应（高度≤420px）========*/
@media (max-height:420px) and (orientation:landscape){
  :root{--js-size:clamp(120px,min(42vw,44vh),240px);--knob-size:calc(var(--js-size)*0.25)}
  .joystick-title{font-size:0.72rem}
  .header h1{font-size:0.82rem}
  .header{padding:3px 8px}
  .header h1{margin-bottom:2px}
  .status-bar{margin-top:2px;gap:4px}
  .status-item{font-size:0.58rem;padding:2px 4px}
  .button{font-size:0.68rem;padding:5px 3px}
  .button-icon{font-size:0.88rem;margin-bottom:2px}
}
/*======== 版权页脚 ========*/
.footer{text-align:center;font-size:0.5rem;color:rgba(255,255,255,.25);padding:0;flex-shrink:0;line-height:0.8;}
.footer a{color:rgba(255,255,255,.3);text-decoration:none}
.footer a:hover{color:rgba(255,255,255,.55)}
</style>
</head>
<body>
<div class="container">
  <!-- 顶部状态栏 -->
  <div class="header">
    <h1>琛光无人机网页遥控器</h1>
    <div class="status-bar">
      <div class="status-item"><span class="status-dot" id="status-dot"></span><span id="connection-text">连接中...</span></div>
      <div class="status-item" id="armed-status-item" style="background:rgba(255,51,51,0.15)"><span id="armed-status" style="color:#ff6666">已上锁</span></div>
      <div class="status-item"><span>飞行模式</span><span id="flight-mode">自稳</span></div>
      <div class="status-item"><span>电池电压</span><span id="battery">-</span></div>
      <div class="status-item"><span>遥控延迟</span><span id="latency">-</span></div>
      <div class="status-item"><span>丢包率</span><span id="packet-loss">0%</span></div>
      <div class="status-item"><span>油门:</span><span id="left-y">0</span><span>%</span></div>
      <div class="status-item"><span>偏航:</span><span id="left-x">0</span></div>
      <div class="status-item"><span>横滚:</span><span id="right-x">0</span></div>
      <div class="status-item"><span>俯仰:</span><span id="right-y">0</span></div>
    </div>
  </div>

  <div class="content">
    <!-- 左手区：油门推拉杆 + 偏航滑杆 -->
    <div class="joystick-container" id="panel-left">
      <div class="joystick-title">油门 / 偏航</div>
      <div class="stick-panel">
        <div class="thr-row">
          <div class="stick-cap">油门 ↑</div>
          <div class="thr-track" id="thr-track">
            <div class="thr-fill" id="thr-fill"></div>
            <div class="thr-notch"></div>
            <div class="thr-thumb" id="thr-thumb"></div>
          </div>
          <div class="stick-read" id="thr-read">0%</div>
        </div>
        <div class="yaw-row">
          <div class="stick-cap">偏航</div>
          <div class="yaw-track" id="yaw-track">
            <div class="yaw-center"></div>
            <div class="yaw-thumb" id="yaw-thumb"></div>
          </div>
          <div class="stick-read" id="yaw-read">0</div>
        </div>
      </div>
    </div>

    <!-- 按钮区 -->
    <div class="buttons-container">
      <div class="buttons-grid" id="buttons-container"></div>
    </div>

    <!-- 右摇杆：前后左右 -->
    <div class="joystick-container" id="panel-right">
      <div class="joystick-title">右摇杆 (俯仰/横滚)</div>
      <div class="joystick-wrapper">
        <div class="joystick" id="joystick-right"><div class="joystick-knob" id="knob-right"></div></div>
      </div>
    </div>
  </div>

  <!-- 调试控制台面板（默认隐藏，点击调试按钮打开） -->
  <div id="console-panel" class="console-panel" style="display:none">
    <div id="console-output" class="console-output"></div>
    <div style="display:flex;gap:6px;touch-action:auto">
      <input id="console-input" placeholder="输入命令 (ps/imu/rc/arm/disarm/help)..."
        style="flex:1;background:rgba(0,0,0,.6);border:1px solid rgba(100,100,100,.5);border-radius:6px;color:#0f8;padding:4px 8px;font-size:0.7rem;font-family:'Courier New',monospace;touch-action:auto">
      <button onclick="sendConsoleCmd()" style="background:#1a73e8;border:none;border-radius:6px;color:#fff;padding:4px 10px;font-size:0.7rem;cursor:pointer;touch-action:auto">发送</button>
    </div>
    <div style="display:flex;gap:6px;touch-action:auto">
      <button onclick="copyConsole()" style="flex:1;background:#2e7d32;border:none;border-radius:6px;color:#fff;padding:5px 8px;font-size:0.7rem;cursor:pointer;touch-action:auto">复制全部输出</button>
      <button onclick="downloadLog()" style="flex:1;background:#6a1b9a;border:none;border-radius:6px;color:#fff;padding:5px 8px;font-size:0.7rem;cursor:pointer;touch-action:auto">下载日志CSV</button>
    </div>
  </div>
  <!-- 版权页脚 -->
  <div class="footer"><a href="https://oshwhub.com/songge8/project_qqqyfdkm" target="_blank">琛光无人机开源项目</a></div>
</div>

<script>
/*======================== 全局变量 ========================*/
let lastAnimationTime = 0;
let connectionOk = false;
let packetStats = { sent: 0, lost: 0 };
let latencyHistory = new Array(10).fill(0);
let latencyIndex = 0;
const SEND_INTERVAL = 50;  // ~20Hz 摇杆检测频率
const FORCE_SEND_INTERVAL = 200; // 静止时强制重发间隔（ms），保持飞控数据新鲜

const touches = new Map();          // 右摇杆多指跟踪：pointerId -> 'right'
let rightStick = {x:0, y:0, rawX:0, rawY:0};
// 左手不再用摇杆，拆成两件独立控件：
//   油门推拉杆 throttleRaw ∈ [-100,100] → 后端映射 0~100%，松手自锁在原位
//   偏航滑杆   yawRaw    ∈ [-100,100] → 左负右正，松手回中（速率型指令）
let throttleRaw = -100;
let yawRaw      = 0;
let thrDragging = false;
let yawDragging = false;

let lastSentValues = { throttle:0, roll:0, pitch:0, yaw:0 };
let lastForceSentTime = 0; // 上次强制重发的时间戳（performance.now()）
let currentValues  = { throttle:0, roll:0, pitch:0, yaw:0 };
const MIN_CHANGE_THRESHOLD = 0.5;

// 固定参数常量（替代前端参数调节面板，与后端 CONFIG_ 对应）
// 死区已移至后端 stickDeadzone/throttleDeadzone 统一处理，前端不再参与
const EXPO    = 40;   // 指数曲线 40%
let consecutiveFails = 0; // 连续失败计数，>=3 才判定断连
let currentFlightMode = 2; // 当前飞行模式编号（与后端同步：2=自稳）

let buttonStates     = new Array(16).fill(false);
let lastButtonStates = new Array(16).fill(false);

let consolePollingTimer = null;
let consoleLastTotal    = 0;   // 增量拉取游标：已展示到第 N 行
let consoleFetchInFlight = false; // 防并发：上次 fetch 未返回时跳过本次
let consolePanelOpen = false;
const CONSOLE_BASE_POLL_MS = 500;
const CONSOLE_CATCHUP_POLL_MS = 80;
const CONSOLE_PAGE_LIMIT = 20;

/*======================== 按钮配置（2×3 六宫格）========================*/
const buttonConfigs = [
  {icon:"🔓",label:"解锁",   color:"#00ff88",desc:"解锁电机"},
  {icon:"🔒",label:"上锁",   color:"#ff3333",desc:"锁定电机"},
  {icon:"🛑",label:"急停",   color:"#ff0055",desc:"紧急停止"},
  {icon:"🔄",label:"切换模式", color:"#00cfff",desc:"自稳→特技→悬停 循环切换"},
  {icon:"🚁",label:"悬停",   color:"#ffd700",desc:"一键悬停空中"},
  {icon:"🖥",label:"调试",   color:"#4a9eff",desc:"调试控制台"}
];

/*======================== 初始化 ========================*/
function init() {
  initButtons();
  initNetwork();
  initPointerEvents();
  initLeftControls();
  requestAnimationFrame(animationLoop);
  requestAnimationFrame(placeControls);
  // 转屏/窗口变化后轨道尺寸改变，需按当前值重新定位两个滑块
  window.addEventListener('resize', placeControls);
  window.addEventListener('orientationchange', placeControls);
}

// 布局就绪前 getBoundingClientRect 会拿到 0，逐帧重试直到能量出尺寸
function placeControls() {
  const track = document.getElementById('thr-track');
  if (!track || track.getBoundingClientRect().height === 0) { requestAnimationFrame(placeControls); return; }
  layoutThrottle();
  layoutYaw();
}

function layoutThrottle() {
  const track = document.getElementById('thr-track');
  const thumb = document.getElementById('thr-thumb');
  const rect  = track.getBoundingClientRect();
  const pad   = thumb.offsetHeight / 2;
  const ratio = (throttleRaw + 100) / 200;                      // 0=最低 1=满油
  const cy    = pad + (1 - ratio) * (rect.height - 2 * pad);   // 旋钮中心 y（自上而下）
  thumb.style.top = cy + 'px';
  // 填充条直接表示真实油门百分比：顶端恰好落在旋钮中心略上方
  document.getElementById('thr-fill').style.height = (ratio * 100).toFixed(1) + '%';
}

function layoutYaw() {
  const track = document.getElementById('yaw-track');
  const thumb = document.getElementById('yaw-thumb');
  const rect  = track.getBoundingClientRect();
  const pad   = thumb.offsetWidth / 2;
  const ratio = (yawRaw + 100) / 200;                           // 0=最左 1=最右
  thumb.style.left = (pad + ratio * (rect.width - 2 * pad)) + 'px';
}

function initButtons() {
  const container = document.getElementById('buttons-container');
  container.innerHTML = '';
  buttonConfigs.forEach((cfg, idx) => {
    const btn = document.createElement('button');
    btn.className = 'button';
    btn.id = `btn-${idx}`;
    btn.title = cfg.desc;
    btn.innerHTML = `<div class="button-icon">${cfg.icon}</div><div>${cfg.label}</div>`;
    btn.style.border = `2px solid ${cfg.color}55`;
    btn.addEventListener('pointerdown', e => {
      e.preventDefault();
      handleButton(idx);
      if (navigator.vibrate) navigator.vibrate(20);
    });
    container.appendChild(btn);
  });
}

function initNetwork() {
  updateConnectionStatus(true);
  setInterval(updateNetworkStatus, 2000);
}

function initPointerEvents() {
  const el = document.getElementById('joystick-right');
  el.addEventListener('pointerdown',   e => { e.preventDefault(); handlePointerStart(e); el.setPointerCapture(e.pointerId); });
  el.addEventListener('pointermove',   e => { e.preventDefault(); handlePointerMove(e); });
  el.addEventListener('pointerup',     e => { e.preventDefault(); handlePointerEnd(e); });
  el.addEventListener('pointercancel', e => { e.preventDefault(); handlePointerEnd(e); });
  el.addEventListener('touchstart', e => e.preventDefault());
  el.addEventListener('touchmove',  e => e.preventDefault());
}

/*======================== 左手控件（油门推拉杆 + 偏航滑杆）========================*/
function initLeftControls() {
  const thr = document.getElementById('thr-track');
  thr.addEventListener('pointerdown', e => {
    e.preventDefault();
    thrDragging = true; thr.classList.add('active');
    thr.setPointerCapture(e.pointerId);
    throttleFromPointer(e.clientY);
    sendJoystickData();
  });
  thr.addEventListener('pointermove', e => { if (thrDragging) { e.preventDefault(); throttleFromPointer(e.clientY); } });
  const endThrottle = e => {
    if (!thrDragging) return;
    thrDragging = false; thr.classList.remove('active');
    if (navigator.vibrate && throttleRaw === -100) navigator.vibrate(15); // 拉到底给一次震动
    sendJoystickData();   // 油门自锁在当前位置，不回零
  };
  thr.addEventListener('pointerup', endThrottle);
  thr.addEventListener('pointercancel', endThrottle);
  thr.addEventListener('touchstart', e => e.preventDefault());
  thr.addEventListener('touchmove',  e => e.preventDefault());

  const yaw = document.getElementById('yaw-track');
  yaw.addEventListener('pointerdown', e => {
    e.preventDefault();
    yawDragging = true; yaw.classList.add('active');
    yaw.setPointerCapture(e.pointerId);
    yawFromPointer(e.clientX);
    sendJoystickData();
  });
  yaw.addEventListener('pointermove', e => { if (yawDragging) { e.preventDefault(); yawFromPointer(e.clientX); } });
  const endYaw = e => {
    if (!yawDragging) return;
    yawDragging = false; yaw.classList.remove('active');
    setYawRaw(0, true);   // 松手回中：偏航是速率型指令
    sendJoystickData();
  };
  yaw.addEventListener('pointerup', endYaw);
  yaw.addEventListener('pointercancel', endYaw);
  yaw.addEventListener('touchstart', e => e.preventDefault());
  yaw.addEventListener('touchmove',  e => e.preventDefault());
}

// 手指纵向位置 → 油门 [-100,100]；靠近底部时吸附到 0，保证能真正收油到底
function throttleFromPointer(clientY) {
  const track = document.getElementById('thr-track');
  const thumb = document.getElementById('thr-thumb');
  const rect  = track.getBoundingClientRect();
  const pad   = thumb.offsetHeight / 2;
  let ratio = 1 - (clientY - rect.top - pad) / Math.max(1, rect.height - 2 * pad);
  ratio = Math.max(0, Math.min(1, ratio));
  if (ratio < 0.03) ratio = 0;
  throttleRaw = Math.round(-100 + 200 * ratio);
  layoutThrottle();
}

// 手指横向位置 → 偏航 [-100,100]，右为正；中段 |值|<5 吸附到 0
function yawFromPointer(clientX) {
  const track = document.getElementById('yaw-track');
  const thumb = document.getElementById('yaw-thumb');
  const rect  = track.getBoundingClientRect();
  const pad   = thumb.offsetWidth / 2;
  let ratio = (clientX - rect.left - pad) / Math.max(1, rect.width - 2 * pad);
  ratio = Math.max(0, Math.min(1, ratio));
  let v = Math.round(-100 + 200 * ratio);
  if (Math.abs(v) < 5) v = 0;
  setYawRaw(v, false);
}

function setYawRaw(value, animate) {
  yawRaw = value;
  const thumb = document.getElementById('yaw-thumb');
  if (animate) {
    thumb.style.transition = 'left .18s ease-out';
    setTimeout(() => { thumb.style.transition = ''; }, 200);
  }
  layoutYaw();
  processJoystickInput();
}

/*======================== 动画循环 ========================*/
function animationLoop(timestamp) {
  if (timestamp - lastAnimationTime >= SEND_INTERVAL) {
    processJoystickInput();
    checkAndSendChanges();
    updateDisplayAll();
    lastAnimationTime = timestamp;
  }
  requestAnimationFrame(animationLoop);
}

/*======================== 摇杆曲线处理 ========================*/
// 仅做 expo 曲线，死区由后端 stickDeadzone 统一处理（对齐 SBUS 模式）
function applyCurve(value) {
  const absVal = Math.abs(value);
  const expoFactor = EXPO / 100;
  let curved = absVal * (1 - expoFactor) + Math.pow(absVal, 3) * expoFactor;
  return curved * (value >= 0 ? 1 : -1);
}

/*======================== 摇杆数据处理 ========================*/
function processJoystickInput() {
  // 发送原始值，后端统一完成映射（对齐SBUS/MAVLink模式，避免双重映射）
  // 油门：throttleRaw∈[-100,+100]，后端 processThrottle: (raw+100)/(2*RAW_MAX)*100 → 0~100%
  // 姿态轴：归一化到[-1,1]做指数曲线后还原×100，后端除以RAW_MAX得[-1,1]
  currentValues.throttle = throttleRaw;
  currentValues.yaw   = applyCurve(yawRaw / 100) * 100;         // 右滑=顺时针，与MAVLink一致
  currentValues.pitch = applyCurve(rightStick.rawY / 100) * 100;  // 上推=前进，与MAVLink一致
  currentValues.roll  = applyCurve(rightStick.rawX / 100) * 100;  // 右推=右滚，与MAVLink一致
  rightStick.x = currentValues.roll;
  rightStick.y = currentValues.pitch;
}

function hasSignificantChange(nv) {
  return Math.abs(nv.throttle - lastSentValues.throttle) > MIN_CHANGE_THRESHOLD ||
         Math.abs(nv.roll     - lastSentValues.roll)     > MIN_CHANGE_THRESHOLD ||
         Math.abs(nv.pitch    - lastSentValues.pitch)    > MIN_CHANGE_THRESHOLD ||
         Math.abs(nv.yaw      - lastSentValues.yaw)      > MIN_CHANGE_THRESHOLD;
}

function checkAndSendChanges() {
  const now = performance.now();
  // 有变化立即发；或超过强制重发间隔时也发一次（保持飞控侧数据新鲜，避免超时断连）
  if (hasSignificantChange(currentValues) || (now - lastForceSentTime >= FORCE_SEND_INTERVAL)) {
    sendJoystickData();
    lastForceSentTime = now;
  }
  for (let i = 0; i < buttonStates.length; i++) {
    if (buttonStates[i] !== lastButtonStates[i]) {
      sendButtonData(i, buttonStates[i]);
      lastButtonStates[i] = buttonStates[i];
    }
  }
}

/*======================== 数据发送函数 ========================*/
function sendJoystickData() {
  sendToESP('/web_rc', { t:1, th:Math.round(currentValues.throttle), r:Math.round(currentValues.roll),
    p:Math.round(currentValues.pitch), y:Math.round(currentValues.yaw), ts:performance.now() });
  lastSentValues = {...currentValues};
  packetStats.sent++;
}

function sendButtonData(buttonIndex, state) {
  sendToESP('/web_rc', { t:2, b:buttonIndex, s:state ? 1 : 0, ts:performance.now() });
}

/*======================== 显示更新 ========================*/
function updateDisplayAll() {
  const thrPct = Math.round((currentValues.throttle + 100) / 2);
  document.getElementById('left-y').textContent  = thrPct;
  document.getElementById('left-x').textContent  = Math.round(currentValues.yaw);
  document.getElementById('right-x').textContent = Math.round(rightStick.x);
  document.getElementById('right-y').textContent = Math.round(rightStick.y);
  document.getElementById('thr-read').textContent = thrPct + '%';
  document.getElementById('yaw-read').textContent = Math.round(currentValues.yaw);
}

/*======================== 右摇杆 Pointer Events 处理 ========================*/
function handlePointerStart(e) {
  touches.set(e.pointerId, 'right');
  document.getElementById('joystick-right').classList.add('active');
  updateJoystickPosition(e.clientX, e.clientY);
}

function handlePointerMove(e) {
  if (touches.get(e.pointerId) === 'right') updateJoystickPosition(e.clientX, e.clientY);
}

function handlePointerEnd(e) {
  if (touches.get(e.pointerId) !== 'right') return;
  touches.delete(e.pointerId);
  const knob     = document.getElementById('knob-right');
  const joystick = document.getElementById('joystick-right');
  joystick.classList.remove('active');
  knob.style.transition = 'transform 0.2s ease-out';
  knob.style.transform  = 'translate(-50%,-50%)';   // 俯仰/横滚恒归中
  setTimeout(() => { knob.style.transition = ''; }, 200);
  rightStick = {x:0, y:0, rawX:0, rawY:0};
  processJoystickInput();
  sendJoystickData();
}

function updateJoystickPosition(clientX, clientY) {
  const joystick = document.getElementById('joystick-right');
  const knob     = document.getElementById('knob-right');
  const rect     = joystick.getBoundingClientRect();
  const cx = rect.width / 2, cy = rect.height / 2;
  const radius = cx - 10;
  let dx = (clientX - rect.left) - cx;
  let dy = (clientY - rect.top)  - cy;
  const dist = Math.sqrt(dx*dx + dy*dy);
  if (dist > radius) { dx = dx/dist*radius; dy = dy/dist*radius; }
  knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
  rightStick.rawX =  dx/radius*100;
  rightStick.rawY = -dy/radius*100;
}

/*======================== 网络处理 ========================*/
function sendToESP(url, data) {
  const t0 = performance.now();
  // 1.5s 超时：避免单请求挂起（后端 handleClient 阻塞时）长期占用连接、污染延迟统计
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 1500);
  fetch(url, { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(data), signal: controller.signal })
    .then(r => { clearTimeout(timeoutId); if (!r.ok) throw new Error(); updateLatency(performance.now() - t0); return r.json(); })
    .then(resp => {
      consecutiveFails = 0;
      updateConnectionStatus(true);
      const names = ['直控','特技','自稳','悬停','自动'];

      // 模式切换结果
      if (resp.m !== undefined) {
        if (resp.m !== currentFlightMode) {
          if (!resp.warn) showToast('✅ 已切换：' + (names[resp.m] || '未知'));
          // 切到悬停/定高：油门杆归中位（中=保持当前高度）；其他模式：归最低（0%）
          if (!thrDragging) setThrottle(resp.m === 3 ? 0 : -100);
        }
        currentFlightMode = resp.m;
        document.getElementById('flight-mode').textContent = names[resp.m] || '自稳';
      }

      // ARM 状态更新（所有响应都同步显示）
      if (resp.arm !== undefined) {
        const el   = document.getElementById('armed-status');
        const item = document.getElementById('armed-status-item');
        el.textContent = resp.arm ? '已解锁' : '已上锁';
        item.style.background = resp.arm ? 'rgba(0,255,136,0.15)' : 'rgba(255,51,51,0.15)';
        el.style.color = resp.arm ? '#00ff88' : '#ff6666';
      }

      // 按钮松开确认 toast（rt=2, bs=0）
      if (resp.rt === 2 && resp.bs === 0) {
        if (!resp.warn) {
          if (resp.bi === 0)
            showToast(resp.arm ? '✅ 已解锁' : '❌ 解锁失败');
          else if (resp.bi === 1)
            showToast(resp.arm ? '⚠️ 上锁失败' : '🔒 已上锁');
          else if (resp.bi === 2)
            showToast('🛑 电机已停止');
        }
        if (resp.warn) showToast('⚠️ ' + resp.warn);
      }
      // 心跳包携带的系统警告（低电自动上锁等），不与按钮 toast 冲突
      if (resp.rt === 4 && resp.warn) showToast('⚠️ ' + resp.warn);
    })
    .catch(() => {
      clearTimeout(timeoutId);
      if (++consecutiveFails >= 3) updateConnectionStatus(false);
    });
}

function updateLatency(latency) {
  // 丢弃异常耗时（>2s 视为后端阻塞/挂起，不计入），避免一个挂起请求把 10 次平均拉到几万 ms
  if (latency > 2000) return;
  latencyHistory[latencyIndex] = latency;
  latencyIndex = (latencyIndex + 1) % latencyHistory.length;
  const valid = latencyHistory.filter(v => v > 0);
  const avg = valid.length ? valid.reduce((a,b)=>a+b) / valid.length : 0;
  document.getElementById('latency').textContent = Math.round(avg) + 'ms';
  const dot = document.querySelector('.status-dot');
  dot.className = avg > 200 ? 'status-dot warning' : avg > 100 ? 'status-dot warning' : 'status-dot connected';
}

function updateNetworkStatus() {
  const lr = packetStats.sent > 0 ? (packetStats.lost/packetStats.sent*100).toFixed(1) : '0';
  document.getElementById('packet-loss').textContent = lr + '%';
  const sc = new AbortController();
  const st = setTimeout(() => sc.abort(), 1500);
  fetch('/web_rc/status', { signal: sc.signal }).then(r=>r.json()).then(d => {
    clearTimeout(st);
    document.getElementById('battery').textContent = (d.voltage !== undefined && d.voltage !== null && d.voltage > 0.5) ? parseFloat(d.voltage).toFixed(2) + 'V' : '-';
  }).catch(()=>{ clearTimeout(st); document.getElementById('battery').textContent = '-'; });
}

function updateConnectionStatus(connected) {
  connectionOk = connected;
  const dot  = document.getElementById('status-dot');
  const text = document.getElementById('connection-text');
  if (connected) { dot.className='status-dot connected'; text.textContent='已连接'; text.style.color='#0f8'; }
  else           { dot.className='status-dot disconnected'; text.textContent='连接断开'; text.style.color='#f33'; }
}
/*======================== 油门位置设置（模式切换等程序化归位）========================*/
function setThrottle(targetRaw) {
  throttleRaw = Math.max(-100, Math.min(100, Math.round(targetRaw)));
  const thumb = document.getElementById('thr-thumb');
  const fill  = document.getElementById('thr-fill');
  const track = document.getElementById('thr-track');
  thumb.style.transition = 'top .3s ease-out';
  fill.style.transition  = 'height .3s ease-out';
  setTimeout(() => { thumb.style.transition = ''; fill.style.transition = ''; }, 320);
  if (track.getBoundingClientRect().height === 0) return; // 布局未就绪，交给 placeControls 定位
  layoutThrottle();
  processJoystickInput();
  sendJoystickData();
}
/*======================== 按钮处理 ========================*/
function handleButton(idx) {
  if (idx === 5) { toggleConsole(); return; }
  if (idx === 4) {
    showToast('🚁 切换悬停...');
    sendButtonData(8, 1);
    setTimeout(() => sendButtonData(8, 0), 100);
    if (navigator.vibrate) navigator.vibrate(30);
    return;
  }
  if (idx === 3) {
    // 模式循环：自稳(2)→特技(1)→定高(3)→自稳
    // 不在点击时弹 toast，结果完全依赖后端 resp.m 确认后触发
    let nextBit;
    if (currentFlightMode === 2)      nextBit = 7; // STAB→ACRO
    else if (currentFlightMode === 1) nextBit = 8; // ACRO→ALTHOLD
    else                              nextBit = 6; // 其他→STAB
    sendButtonData(nextBit, 1);
    setTimeout(() => sendButtonData(nextBit, 0), 100);
    if (navigator.vibrate) navigator.vibrate(30);
    return;
  }
  // 解锁/上锁/急停：先发按下（state=1），100ms后发松开（state=0）
  // 后端响应中携带 rt/bi/bs，前端用这些字段判断 toast，无需 lastPressedButton
  if (idx === 0 || idx === 1 || idx === 2) {
    if (idx === 0)      showToast('🔓 解锁中...');
    else if (idx === 1) showToast('🔒 上锁中...');
    else if (idx === 2) showToast('🛑 急停指令发送中...');
    sendButtonData(idx, 1);
    setTimeout(() => sendButtonData(idx, 0), 100);
    if (navigator.vibrate) navigator.vibrate(30);
  }
}

/*======================== Toast 提示 ========================*/
function showToast(msg) {
  let t = document.getElementById('toast-msg');
  if (!t) {
    t = document.createElement('div');
    t.id = 'toast-msg';
    t.style.cssText = 'position:fixed;top:50%;left:50%;transform:translate(-50%,-50%);background:rgba(0,0,0,.85);color:#fff;padding:12px 20px;border-radius:10px;font-size:14px;z-index:9999;pointer-events:none;text-align:center;max-width:80vw;border:1px solid rgba(255,255,255,.2);transition:opacity .3s';
    document.body.appendChild(t);
  }
  t.textContent = msg; t.style.opacity = '1';
  clearTimeout(t._timer);
  t._timer = setTimeout(() => { t.style.opacity = '0'; }, 2000);
}

/*======================== 调试控制台 ========================*/
function toggleConsole() {
  const panel = document.getElementById('console-panel');
  const btn   = document.getElementById('btn-5');
  const open  = panel.style.display === 'none';
  consolePanelOpen = open;
  panel.style.display = open ? 'flex' : 'none';
  if (open) { panel.style.flexDirection = 'column'; }
  if (btn) { open ? btn.classList.add('active') : btn.classList.remove('active'); }
  if (open) {
    document.getElementById('console-output').innerHTML = '';
    consoleLastTotal = 0;
    fetch('/console/enable', {method:'POST'}).catch(()=>{});
    fetchConsoleLogs();
  } else {
    fetch('/console/disable', {method:'POST'}).catch(()=>{});
    clearTimeout(consolePollingTimer);
    consolePollingTimer = null;
  }
}

function scheduleConsolePoll(delayMs) {
  if (!consolePanelOpen) return;
  clearTimeout(consolePollingTimer);
  consolePollingTimer = setTimeout(fetchConsoleLogs, delayMs);
}

function fetchConsoleLogs() {
  if (!consolePanelOpen) return;
  if (consoleFetchInFlight) {
    scheduleConsolePoll(CONSOLE_CATCHUP_POLL_MS);
    return;
  }

  consoleFetchInFlight = true;
  fetch('/console?since=' + consoleLastTotal + '&limit=' + CONSOLE_PAGE_LIMIT).then(r=>r.json()).then(data => {
    const out = document.getElementById('console-output');
    if (data.lines && data.lines.length > 0) {
      const frag = document.createDocumentFragment();
      data.lines.forEach(l => {
        const div = document.createElement('div');
        div.textContent = l;
        frag.appendChild(div);
      });
      out.appendChild(frag);
      out.scrollTop = out.scrollHeight;
      // 限制 DOM 行数，避免长时间运行内存泄漏
      while (out.children.length > 200) out.removeChild(out.firstChild);
    }

    if (typeof data.next === 'number') consoleLastTotal = data.next;
    else if (typeof data.total === 'number') consoleLastTotal = data.total;

    scheduleConsolePoll(data.has_more ? CONSOLE_CATCHUP_POLL_MS : CONSOLE_BASE_POLL_MS);
  }).catch(()=>{
    scheduleConsolePoll(CONSOLE_BASE_POLL_MS);
  }).finally(() => {
    consoleFetchInFlight = false;
  });
}

/* 复制控制台全部输出：优先用旧版 execCommand（http 下也能用），失败再退回 clipboard API */
function copyConsole() {
  const el = document.getElementById('console-output');
  const text = (el.innerText || el.textContent || '').trim();
  if (!text) { showToast('没有可复制的内容'); return; }
  let ok = false;
  const ta = document.createElement('textarea');
  ta.value = text;
  ta.setAttribute('readonly', '');
  ta.style.position = 'fixed';
  ta.style.top = '0';
  ta.style.left = '-9999px';
  document.body.appendChild(ta);
  ta.select();
  ta.setSelectionRange(0, text.length);
  try { ok = document.execCommand('copy'); } catch (e) { ok = false; }
  document.body.removeChild(ta);
  if (ok) { showToast('已复制 ' + text.length + ' 字符'); return; }
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(text).then(()=>showToast('已复制 ' + text.length + ' 字符'))
      .catch(()=>showToast('复制失败：请长按输出区手动选择'));
  } else {
    showToast('复制失败：请长按输出区手动选择');
  }
}

/* 打开黑匣子精简摘要页（最近 40 行 ≈ 0.4 秒，9 列紧凑格式，手机可直接截图看） */
function downloadLog() {
  window.open('/log.txt?n=40', '_blank');
}

function sendConsoleCmd() {
  const input = document.getElementById('console-input');
  const cmd = input.value.trim();
  if (!cmd) return;
  input.value = '';
  fetch('/console/cmd', {method:'POST', headers:{'Content-Type':'text/plain'}, body:cmd})
    .then(r => r.json())
    .then(resp => {
      if (!resp.ok) {
        if (resp.e === 'queue full') showToast('⚠️ 命令队列已满，请稍后重试');
        return;
      }
      fetchConsoleLogs();
    }).catch(()=>{});
}

// 回车发送 + ↑/↓ 命令历史
(function(){
  const cmdHistory = [];
  let historyIdx   = -1;
  document.getElementById('console-input').addEventListener('keydown', function(e) {
    if (e.key === 'Enter') {
      e.preventDefault();
      const v = this.value.trim();
      if (v) { cmdHistory.unshift(v); if (cmdHistory.length > 20) cmdHistory.pop(); }
      sendConsoleCmd();
      historyIdx = -1;
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      if (historyIdx < cmdHistory.length - 1) { historyIdx++; this.value = cmdHistory[historyIdx]; }
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      if (historyIdx > 0) { historyIdx--; this.value = cmdHistory[historyIdx]; }
      else { historyIdx = -1; this.value = ''; }
    }
  });
})();

/*======================== 事件绑定 ========================*/
document.addEventListener('DOMContentLoaded', init);
document.addEventListener('contextmenu', e => e.preventDefault());

// 心跳：2000ms，连续3次失败才判定断连
setInterval(() => {
  if (connectionOk) sendToESP('/web_rc/heartbeat', {t:4, ts:performance.now()});
}, 2000);
</script>
</body>
</html>
)rawliteral";

#endif
