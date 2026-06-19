#ifndef REMOTE_PAGE_H
#define REMOTE_PAGE_H

#define REMOTE_HTML \
"<!DOCTYPE html><html lang='zh'><head><meta charset='UTF-8'>" \
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no,viewport-fit=cover'>" \
"<title>遥控器</title><style>" \
"*{margin:0;padding:0;box-sizing:border-box;user-select:none;-webkit-user-select:none}" \
"body{background:#0a0a14;color:#ddd;font-family:system-ui;height:100vh;display:flex;flex-direction:column;overflow:hidden;touch-action:manipulation}" \
".top{display:flex;align-items:center;justify-content:space-between;padding:8px 12px;background:#111;gap:8px}" \
".top .title{font-size:14px;font-weight:700;color:#4fc3f7}" \
".dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:4px}" \
".dot.on{background:#0f0;box-shadow:0 0 6px #0f0}.dot.off{background:#f44}" \
".status-text{font-size:11px;color:#888}" \
".btn-row{display:flex;gap:6px;padding:6px 12px;flex-wrap:wrap}" \
".btn{flex:1;min-width:55px;padding:10px 6px;border:none;border-radius:6px;font-size:11px;font-weight:700;cursor:pointer;text-align:center}" \
".btn:active{opacity:0.7}" \
".btn-enable{background:#1b5e20;color:#4caf50}" \
".btn-enable.active{background:#4caf50;color:#000}" \
".btn-disable{background:#b71c1c;color:#ef5350}" \
".btn-mode{background:#1a237e;color:#7986cb}" \
".btn-mode.sel{background:#3949ab;color:#fff;border:2px solid #5c6bc0}" \
".btn-light{background:#f9a825;color:#000}" \
".btn-light.on{background:#ffeb3b}" \
".btn-stop{background:#c62828;color:#fff;font-size:13px}" \
".joy-area{flex:1;display:flex;position:relative;touch-action:none;min-height:0}" \
".joy-half{flex:1;position:relative;display:flex;align-items:center;justify-content:center;touch-action:none}" \
".joy-half.left{border-right:1px solid #222;background:radial-gradient(ellipse at center,#1a1a2e 0%,#0a0a14 70%)}" \
".joy-half.right{background:radial-gradient(ellipse at center,#1a1a2e 0%,#0a0a14 70%)}" \
".cross,.cross2{position:absolute;width:4px;height:70%;background:#1a1a3a;border-radius:2px;pointer-events:none}" \
".cross2{width:70%;height:4px}" \
".thumb{width:70px;height:70px;border-radius:50%;position:absolute;z-index:2;transition:transform 0.03s linear;pointer-events:none}" \
".thumb-fwd{background:radial-gradient(circle,#e94560 30%,#a02 100%);box-shadow:0 0 20px rgba(233,69,96,0.4)}" \
".thumb-yaw{background:radial-gradient(circle,#4fc3f7 30%,#069 100%);box-shadow:0 0 20px rgba(79,195,247,0.4)}" \
".thumb-label{position:absolute;bottom:8px;font-size:10px;color:#555;pointer-events:none;text-transform:uppercase;letter-spacing:2px}" \
".value-indicator{position:absolute;top:10px;font-size:11px;font-family:monospace;color:#888;pointer-events:none}" \
".bot{display:flex;justify-content:center;align-items:center;padding:6px 12px;background:#111;gap:8px}" \
".bot .info{font-size:10px;color:#555;font-family:monospace}" \
"</style></head><body>" \
"<div class='top'>" \
 "<span class='title'>遥控器</span>" \
 "<span><span class='dot off' id='ws_dot'></span><span class='status-text' id='ws_text'>离线</span></span>" \
"</div>" \
"<div class='btn-row'>" \
 "<button class='btn btn-enable' id='btn_enable' onclick='sendCmd(\"enable\")'>使能</button>" \
 "<button class='btn btn-disable' id='btn_disable' onclick='sendCmd(\"disable\")'>关闭</button>" \
 "<button class='btn btn-mode' id='btn_stand' onclick='setMode(\"standing\")'>站立</button>" \
 "<button class='btn btn-mode sel' id='btn_drive' onclick='setMode(\"driving\")'>行驶</button>" \
 "<button class='btn btn-mode' id='btn_sit' onclick='setMode(\"sitting\")'>坐下</button>" \
"</div>" \
"<div class='joy-area'>" \
 "<div class='joy-half left' id='zone_fwd'>" \
  "<div class='cross'></div><div class='cross2'></div>" \
  "<div class='thumb thumb-fwd' id='thumb_fwd'></div>" \
  "<div class='thumb-label'>前进/后退</div>" \
  "<div class='value-indicator' id='val_fwd'>0</div>" \
 "</div>" \
 "<div class='joy-half right' id='zone_yaw'>" \
  "<div class='cross'></div><div class='cross2'></div>" \
  "<div class='thumb thumb-yaw' id='thumb_yaw'></div>" \
  "<div class='thumb-label'>左转/右转</div>" \
  "<div class='value-indicator' id='val_yaw'>0</div>" \
 "</div>" \
"</div>" \
"<div class='bot'>" \
 "<button class='btn btn-light' id='btn_light' onclick='toggleLight()'>灯</button>" \
 "<button class='btn btn-stop' onclick='emergencyStop()'>紧急停止</button>" \
 "<span class='info' id='info_text'></span>" \
"</div>" \
"<script>" \
"var ws=null,wsOk=false,fwd=0,yaw=0,lightOn=false,currentMode='driving',enabled=false;" \
"var thFwd=document.getElementById('thumb_fwd'),thYaw=document.getElementById('thumb_yaw');" \
"var zFwd=document.getElementById('zone_fwd'),zYaw=document.getElementById('zone_yaw');" \
"var vFwd=document.getElementById('val_fwd'),vYaw=document.getElementById('val_yaw');" \
"var infoText=document.getElementById('info_text');" \
"function setStatus(on,txt){" \
 "wsOk=on;var d=document.getElementById('ws_dot'),t=document.getElementById('ws_text');" \
 "d.className='dot '+(on?'on':'off');t.textContent=txt||(on?'已连接':'离线');" \
"}" \
"function connect(){" \
 "var url='ws://'+window.location.hostname+'/ws';" \
 "try{ws=new WebSocket(url);}catch(e){setStatus(false,'连接失败');setTimeout(connect,2000);return;}" \
 "ws.onopen=function(){setStatus(true,'已连接');};" \
 "ws.onclose=function(){setStatus(false,'断开');setTimeout(connect,2000);};" \
 "ws.onerror=function(){ws.close();};" \
 "ws.onmessage=function(e){if(e.data instanceof Blob)return;try{var j=JSON.parse(e.data);if(j.type==='state')infoText.textContent=j.data.freq+'Hz';}catch(_){}};" \
"}" \
"function sendRaw(obj){if(wsOk)ws.send(JSON.stringify(obj));}" \
"function sendCmd(cmd){sendRaw({type:'command',cmd:cmd});if(cmd==='enable'){enabled=true;updateEnableBtn();}else if(cmd==='disable'){enabled=false;updateEnableBtn();}}" \
"function setMode(m){" \
 "currentMode=m;" \
 "['standing','driving','sitting'].forEach(function(id){" \
  "var b=document.getElementById('btn_'+id.substring(0,4));" \
  "if(b)b.className='btn btn-mode'+(m===id?' sel':'');" \
 "});" \
 "sendRaw({type:'command',cmd:'set_mode',mode:m});" \
"}" \
"function sendDrive(f,y){" \
 "if(!wsOk)return;" \
 "if(!enabled){infoText.textContent='请先使能';return;}" \
 "sendRaw({type:'drive',fwd:f,yaw:y});" \
 "infoText.textContent='F:'+f+' Y:'+y;" \
"}" \
"function emergencyStop(){fwd=0;yaw=0;thFwd.style.transform='translate(0,0)';thYaw.style.transform='translate(0,0)';sendRaw({type:'command',cmd:'disable'});enabled=false;updateEnableBtn();infoText.textContent='紧急停止';}" \
"function toggleLight(){lightOn=!lightOn;var b=document.getElementById('btn_light');b.className='btn btn-light'+(lightOn?' on':'');b.textContent=lightOn?'灯 开':'灯';sendRaw({type:'light',on:lightOn?1:0});}" \
"function updateEnableBtn(){var b=document.getElementById('btn_enable');b.className='btn btn-enable'+(enabled?' active':'');b.textContent=enabled?'已使能':'使能';}" \
"function clamp(v,lo,hi){return v<lo?lo:v>hi?hi:v;}" \
"function setupKnob(zone,thumb,valEl,isVert){" \
 "var down=false,sz=null;" \
 "function updateSz(){sz=zone.getBoundingClientRect();}" \
 "function move(e){" \
  "if(!sz)updateSz();" \
  "var t=e.touches?e.touches[0]:e;" \
  "var cx=sz.left+sz.width/2,cy=sz.top+sz.height/2;" \
  "var dx=(t.clientX-cx)/(sz.width/2),dy=(t.clientY-cy)/(sz.height/2);" \
  "dx=clamp(dx,-1,1);dy=clamp(dy,-1,1);" \
  "var val=isVert?-dy:dx;" \
  "var x=dx*Math.min(sz.width,sz.height)*0.35,y=-dy*Math.min(sz.width,sz.height)*0.35;" \
  "thumb.style.transform='translate('+x+'px,'+y+'px)';" \
  "var scaled=(val*200)|0;" \
  "if(isVert){fwd=scaled;}else{yaw=scaled;}" \
  "valEl.textContent=scaled;" \
  "sendDrive(fwd,yaw);" \
 "}" \
 "function end(e){" \
  "down=false;fwd=0;yaw=0;" \
  "thumb.style.transform='translate(0,0)';" \
  "valEl.textContent='0';" \
  "sendDrive(fwd,yaw);" \
 "}" \
 "zone.addEventListener('touchstart',function(e){e.preventDefault();down=true;updateSz();move(e);},{passive:false});" \
 "zone.addEventListener('touchmove',function(e){e.preventDefault();if(down)move(e);},{passive:false});" \
 "zone.addEventListener('touchend',function(e){e.preventDefault();end(e);},{passive:false});" \
 "zone.addEventListener('touchcancel',function(e){e.preventDefault();end(e);},{passive:false});" \
 "zone.addEventListener('mousedown',function(e){e.preventDefault();down=true;updateSz();move(e);});" \
 "zone.addEventListener('mousemove',function(e){if(down)move(e);});" \
 "zone.addEventListener('mouseup',function(e){end(e);});" \
 "zone.addEventListener('mouseleave',function(e){if(down)end(e);});" \
 "window.addEventListener('resize',function(){updateSz();});" \
"}" \
"setupKnob(zFwd,thFwd,vFwd,true);" \
"setupKnob(zYaw,thYaw,vYaw,false);" \
"var keys={};" \
"function kbDrive(){" \
 "var kf=0,ky=0;" \
 "if(keys['KeyW']||keys['ArrowUp'])kf=200;" \
 "if(keys['KeyS']||keys['ArrowDown'])kf=-200;" \
 "if(keys['KeyA']||keys['ArrowLeft'])ky=-200;" \
 "if(keys['KeyD']||keys['ArrowRight'])ky=200;" \
 "if(kf!==fwd||ky!==yaw){fwd=kf;yaw=ky;" \
  "thFwd.style.transform='translate(0,'+(-kf/200*thFwd.parentElement.getBoundingClientRect().height*0.35)+'px)';" \
  "thYaw.style.transform='translate('+(ky/200*thYaw.parentElement.getBoundingClientRect().width*0.35)+'px,0)';" \
  "vFwd.textContent=kf;vYaw.textContent=ky;" \
  "sendDrive(fwd,yaw);}" \
 "if(!kf&&!ky)requestAnimationFrame(kbDrive);" \
"}" \
"document.addEventListener('keydown',function(e){if(!e.repeat){keys[e.code]=true;kbDrive();}});" \
"document.addEventListener('keyup',function(e){keys[e.code]=false;if(!keys['KeyW']&&!keys['KeyS']&&!keys['KeyA']&&!keys['KeyD']&&!keys['ArrowUp']&&!keys['ArrowDown']&&!keys['ArrowLeft']&&!keys['ArrowRight']){fwd=0;yaw=0;thFwd.style.transform='translate(0,0)';thYaw.style.transform='translate(0,0)';vFwd.textContent='0';vYaw.textContent='0';sendDrive(0,0);}});" \
"connect();" \
"</script></body></html>"

#endif /* REMOTE_PAGE_H */
