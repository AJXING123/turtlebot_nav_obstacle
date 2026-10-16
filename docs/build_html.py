#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert TurtleBot3_Nav2_Project_Report.md to a mobile-friendly single-file HTML."""
import re
import markdown
from markdown.extensions.toc import TocExtension, slugify_unicode
from pathlib import Path

DOCS = Path(__file__).parent
SRC = DOCS / "TurtleBot3_Nav2_Project_Report.md"
OUT = DOCS / "TurtleBot3_Nav2_Project_Report.html"

text = SRC.read_text(encoding="utf-8")

mermaid_blocks = []
def _grab_mermaid(m):
    mermaid_blocks.append(m.group(1))
    return "\n\nMERMAIDPLACEHOLDER%dENDPH\n\n" % (len(mermaid_blocks) - 1)
text = re.sub(r"```mermaid\n(.*?)```", _grab_mermaid, text, flags=re.S)

md = markdown.Markdown(extensions=[
    "tables", "fenced_code", "sane_lists", "attr_list",
    TocExtension(permalink=False, toc_depth="2-3", slugify=slugify_unicode, separator="-"),
])
body = md.convert(text)
toc = md.toc

def _restore(m):
    idx = int(m.group(1))
    code = mermaid_blocks[idx]
    esc = code.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    return '<pre class="mermaid">\n%s</pre>' % esc
body = re.sub(r"<p>MERMAIDPLACEHOLDER(\d+)ENDPH</p>", _restore, body)
body = re.sub(r"MERMAIDPLACEHOLDER(\d+)ENDPH", _restore, body)

CSS = """
:root{
  --bg:#ffffff; --fg:#24292f; --muted:#57606a; --border:#d0d7de;
  --card:#f6f8fa; --accent:#1a73e8; --code-bg:#f6f8fa; --th-bg:#eaeef2;
  --quote-bg:#f0f6ff; --quote-bar:#1a73e8;
}
@media (prefers-color-scheme: dark){
  :root{
    --bg:#0d1117; --fg:#e6edf3; --muted:#9198a1; --border:#3d444d;
    --card:#161b22; --accent:#58a6ff; --code-bg:#161b22; --th-bg:#21262d;
    --quote-bg:#12233b; --quote-bar:#58a6ff;
  }
}
*{box-sizing:border-box}
html{-webkit-text-size-adjust:100%}
body{
  margin:0; background:var(--bg); color:var(--fg);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC","Hiragino Sans GB","Microsoft YaHei",sans-serif;
  font-size:16px; line-height:1.75;
}
.topbar{
  position:sticky; top:0; z-index:50; display:flex; align-items:center; gap:10px;
  padding:calc(10px + env(safe-area-inset-top)) 14px 10px;
  background:var(--accent); color:#fff; box-shadow:0 1px 4px rgba(0,0,0,.25);
}
.topbar .title{font-size:15px; font-weight:600; overflow:hidden; text-overflow:ellipsis; white-space:nowrap}
.menu-btn{flex:0 0 auto; width:38px; height:38px; border:none; border-radius:8px;
  background:rgba(255,255,255,.18); color:#fff; font-size:19px; cursor:pointer}
.sidebar{
  position:fixed; top:0; left:0; bottom:0; width:min(82vw,330px); z-index:100;
  background:var(--bg); border-right:1px solid var(--border);
  overflow-y:auto; -webkit-overflow-scrolling:touch;
  transform:translateX(-105%); transition:transform .25s ease;
  padding:calc(14px + env(safe-area-inset-top)) 12px 30px;
}
.sidebar.open{transform:translateX(0)}
.sidebar h2{font-size:15px; margin:0 0 10px; padding:0 8px; color:var(--muted)}
.sidebar ul{list-style:none; margin:0; padding-left:14px}
.sidebar>div>ul{padding-left:0}
.sidebar li{margin:2px 0}
.sidebar a{display:block; padding:6px 8px; border-radius:7px; color:var(--fg); text-decoration:none; font-size:14px; line-height:1.45}
.sidebar a:active{background:var(--card)}
.sidebar .close-btn{position:absolute; top:calc(10px + env(safe-area-inset-top)); right:10px; width:34px; height:34px; border:none; background:var(--card); color:var(--fg); border-radius:8px; font-size:17px; cursor:pointer}
.overlay{position:fixed; inset:0; background:rgba(0,0,0,.45); z-index:90; opacity:0; pointer-events:none; transition:opacity .25s}
.overlay.show{opacity:1; pointer-events:auto}
main{max-width:860px; margin:0 auto; padding:18px 16px calc(60px + env(safe-area-inset-bottom))}
h1{font-size:1.5em; line-height:1.35; margin:.4em 0 .8em; padding-bottom:.4em; border-bottom:2px solid var(--accent)}
h2{font-size:1.28em; margin:1.8em 0 .7em; padding:.35em .6em; border-left:5px solid var(--accent); background:var(--card); border-radius:0 8px 8px 0}
h3{font-size:1.1em; margin:1.5em 0 .5em; color:var(--accent)}
h4{font-size:1em; margin:1.2em 0 .4em}
p{margin:.7em 0}
a{color:var(--accent)}
hr{border:none; border-top:1px solid var(--border); margin:2em 0}
.table-wrap{overflow-x:auto; -webkit-overflow-scrolling:touch; margin:1em 0; border:1px solid var(--border); border-radius:10px}
table{border-collapse:collapse; width:100%; font-size:13.5px; min-width:520px}
th,td{border:1px solid var(--border); padding:8px 10px; text-align:left; vertical-align:top; line-height:1.55}
th{background:var(--th-bg); font-weight:600; white-space:nowrap}
tr:nth-child(even) td{background:var(--card)}
code{
  font-family:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,"Liberation Mono",monospace;
  font-size:.85em; background:var(--code-bg); border:1px solid var(--border);
  border-radius:5px; padding:.12em .38em; word-break:break-all;
}
pre{
  background:var(--code-bg); border:1px solid var(--border); border-radius:10px;
  padding:12px 14px; overflow-x:auto; -webkit-overflow-scrolling:touch;
  font-size:12.8px; line-height:1.55; margin:1em 0;
}
pre code{background:none; border:none; padding:0; font-size:inherit; word-break:normal; white-space:pre}
blockquote{
  margin:1em 0; padding:.6em 1em; background:var(--quote-bg);
  border-left:4px solid var(--quote-bar); border-radius:0 8px 8px 0; color:var(--fg);
}
blockquote p{margin:.35em 0}
ul,ol{padding-left:1.5em}
li{margin:.3em 0}
pre.mermaid{background:transparent; border:none; text-align:center; overflow-x:auto; font-size:12px}
#backtop{
  position:fixed; right:14px; bottom:calc(18px + env(safe-area-inset-bottom)); z-index:40;
  width:46px; height:46px; border-radius:50%; border:none; cursor:pointer;
  background:var(--accent); color:#fff; font-size:20px;
  box-shadow:0 2px 8px rgba(0,0,0,.35); display:none;
}
.footer{margin-top:3em; padding-top:1em; border-top:1px solid var(--border); color:var(--muted); font-size:13px}
@media (min-width:1024px){
  .sidebar{transform:translateX(0); position:fixed}
  .menu-btn{display:none}
  main{margin-left:min(82vw,330px)}
  body{font-size:16.5px}
}
"""

JS = """
const sb=document.getElementById('sidebar'),ov=document.getElementById('overlay');
function openSb(){sb.classList.add('open');ov.classList.add('show')}
function closeSb(){sb.classList.remove('open');ov.classList.remove('show')}
document.getElementById('menuBtn').onclick=openSb;
document.getElementById('closeBtn').onclick=closeSb;
ov.onclick=closeSb;
sb.querySelectorAll('a').forEach(a=>a.addEventListener('click',closeSb));
const bt=document.getElementById('backtop');
window.addEventListener('scroll',()=>{bt.style.display=window.scrollY>600?'block':'none'},{passive:true});
bt.onclick=()=>window.scrollTo({top:0,behavior:'smooth'});
document.querySelectorAll('table').forEach(t=>{
  if(!t.parentElement.classList.contains('table-wrap')){
    const w=document.createElement('div');w.className='table-wrap';
    t.parentNode.insertBefore(w,t);w.appendChild(t);
  }
});
if(window.mermaid){
  const dark=window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches;
  try{
    mermaid.initialize({startOnLoad:false,theme:dark?'dark':'default',securityLevel:'loose'});
    document.querySelectorAll('pre.mermaid').forEach((el,i)=>{
      const src=el.textContent;
      mermaid.render('mmd'+i,src).then(({svg})=>{el.innerHTML=svg;}).catch(()=>{el.textContent=src;});
    });
  }catch(e){}
}
"""

parts = []
parts.append('<!DOCTYPE html>\n<html lang="zh-CN">\n<head>\n<meta charset="UTF-8">\n')
parts.append('<meta name="viewport" content="width=device-width, initial-scale=1.0, viewport-fit=cover">\n')
parts.append('<meta name="theme-color" content="#1a73e8">\n')
parts.append('<title>TurtleBot3 多点定点导航项目技术报告</title>\n<style>' + CSS + '</style>\n</head>\n<body>\n')
parts.append('<div class="topbar"><button class="menu-btn" id="menuBtn" aria-label="open toc">&#9776;</button>')
parts.append('<span class="title">TurtleBot3 多点定点导航项目技术报告</span></div>\n')
parts.append('<div class="overlay" id="overlay"></div>\n')
parts.append('<nav class="sidebar" id="sidebar"><button class="close-btn" id="closeBtn" aria-label="close toc">&#10005;</button><h2>目 录</h2>\n')
parts.append(toc + '\n</nav>\n<main>\n')
parts.append(body + '\n')
parts.append('<div class="footer">由 TurtleBot3_Nav2_Project_Report.md 自动生成 · 移动端阅读版</div>\n</main>\n')
parts.append('<button id="backtop" aria-label="back to top">&#8593;</button>\n')
parts.append('<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>\n')
parts.append('<script>' + JS + '</script>\n</body>\n</html>\n')

OUT.write_text("".join(parts), encoding="utf-8")
print("OK -> %s (%.0f KB)" % (OUT, OUT.stat().st_size / 1024))
