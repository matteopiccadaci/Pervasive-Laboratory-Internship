import asyncio
from ssl import _create_unverified_context
from typing import Optional
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
from autobahn.asyncio.wamp import ApplicationSession, ApplicationRunner

WAMP_URL = "wss://crossbar:8181/ws"
WAMP_REALM = "s4t"
board_hostname = "husarion"

app = FastAPI()
wamp_session: Optional[ApplicationSession] = None


class WAMPClient(ApplicationSession):
    async def onJoin(self, details):
        global wamp_session
        wamp_session = self
        print(f"[WAMP] Sessione connessa al router WAMP, realm={WAMP_REALM}")


async def start_wamp():
    ssl_context = _create_unverified_context()

    runner = ApplicationRunner(
        url=WAMP_URL,
        realm=WAMP_REALM,
        ssl=ssl_context,
    )
    await runner.run(WAMPClient, start_loop=False)


@app.on_event("startup")
async def startup_event():
    asyncio.create_task(start_wamp())


class KeyPayload(BaseModel):
    key: str


class AnalogPayload(BaseModel):
    lx: float
    rx: float
    turbo: bool = False


@app.post("/teleop/key")
async def teleop_key(payload: KeyPayload):
    if not wamp_session:
        raise HTTPException(status_code=503, detail="WAMP not connected yet")

    key = payload.key

    allowed_keys = set([
        "i", "o", "j", "l", "u", ",", ".", "m",
        "O", "I", "J", "L", "U", "<", ">", "M",
        "t", "b",
        "q", "z", "w", "x", "e", "c",
        "v",
    ])

    if key not in allowed_keys:
        raise HTTPException(status_code=400, detail=f"Key '{key}' not supported")

    procedure = f"iotronic.{board_hostname}.teleop_drive"

    try:
        result = await wamp_session.call(procedure, {"key": key})
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"RPC call error: {e}")
    return {
        "board": board_hostname,
        "key": key,
        "rpc_procedure": procedure,
        "rpc_result": result,
    }


@app.post("/teleop/analog")
async def teleop_analog(payload: AnalogPayload):
    if not wamp_session:
        raise HTTPException(status_code=503, detail="WAMP not connected yet")

    procedure = f"iotronic.{board_hostname}.teleop_analog"

    body = {
        "lx": float(payload.lx),
        "rx": float(payload.rx),
        "turbo": bool(payload.turbo),
    }

    try:
        result = await wamp_session.call(procedure, body)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"RPC call error: {e}")
    return {
        "board": board_hostname,
        "payload": body,
        "rpc_procedure": procedure,
        "rpc_result": result,
    }


@app.get("/", response_class=HTMLResponse)
async def teleop_page():
    html_content = f"""
    <!DOCTYPE html>
    <html lang="it">
    <head>
        <meta charset="UTF-8" />
        <title>Teleop ROSbotXL via RPC</title>
        <style>
            body {{
                font-family: Arial, sans-serif;
                background: #111;
                color: #eee;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                height: 100vh;
                margin: 0;
            }}
            .card {{
                border: 1px solid #444;
                border-radius: 8px;
                padding: 20px 30px;
                max-width: 900px;
                text-align: center;
                background: #1b1b1b;
                box-shadow: 0 0 10px rgba(0,0,0,0.6);
                position: relative;
            }}
            h1 {{
                margin-top: 0;
            }}
            kbd {{
                background: #333;
                border-radius: 4px;
                padding: 4px 8px;
                margin: 2px;
                display: inline-block;
                min-width: 20px;
            }}
            #status {{
                margin-top: 10px;
                font-size: 0.9rem;
            }}
            .ok {{ color: #4caf50; }}
            .err {{ color: #f44336; }}
            .log {{
                margin-top: 15px;
                font-size: 0.85rem;
                max-height: 180px;
                overflow-y: auto;
                text-align: left;
                border-top: 1px solid #333;
                padding-top: 10px;
            }}
            .small {{
                font-size: 0.8rem;
                color: #bbb;
            }}
            .mode-switch {{
                margin-bottom: 15px;
            }}
            .mode-switch button {{
                padding: 6px 14px;
                margin: 0 5px;
                border-radius: 6px;
                border: 1px solid #555;
                background: #222;
                color: #eee;
                cursor: pointer;
            }}
            .mode-switch button.active {{
                background: #4caf50;
                border-color: #4caf50;
                color: #111;
            }}
            .hidden {{
                display: none;
            }}

            .joysticks {{
                position: fixed;
                bottom: 40px;
                left: 0;
                right: 0;
                pointer-events: none; 
            }}
            .joystick-container {{
                width: 180px;
                height: 180px;
                border-radius: 50%;
                border: 2px solid #555;
                position: fixed;
                bottom: 40px;
                background: rgba(255,255,255,0.02);
                pointer-events: auto; /* questo sì */
                touch-action: none;
            }}
            #leftJoy {{
                left: 40px;
            }}
            #rightJoy {{
                right: 40px;
            }}
            .joystick-knob {{
                width: 75px;
                height: 75px;
                border-radius: 50%;
                background: #4caf50;
                position: absolute;
                left: 50%;
                top: 50%;
                transform: translate(-50%, -50%);
            }}
        </style>
    </head>
    <body>
        <div class="card">
            <h1>Teleop ROSbotXL</h1>
            <p class="small">Board target: <strong>{board_hostname}</strong></p>

            <div class="mode-switch">
                <button id="btnKeyboard" class="active">Keyboard</button>
                <button id="btnAnalog">Joypad</button>
            </div>

            <div id="keyboardHelp">
                <p>Use the keys to drive the ROSbotXL (teleop via RPC WAMP).</p>

                <p>
                    <strong>Movement:</strong><br/>
                    <kbd>u</kbd> <kbd>i</kbd> <kbd>o</kbd><br/>
                    <kbd>j</kbd> <kbd>k</kbd> <kbd>l</kbd><br/>
                    <kbd>m</kbd> <kbd>,</kbd> <kbd>.</kbd>
                </p>
                <p>
                    <strong>Speed:</strong> <kbd>q</kbd> <kbd>w</kbd> <kbd>e</kbd> /
                    <kbd>z</kbd> <kbd>x</kbd> <kbd>c</kbd><br/>
                    <strong>Stop teleop:</strong> <kbd>v</kbd>
                </p>
            </div>

            <div id="analogHelp" class="hidden">
                <p>Move the two virtual joysticks to control the ROSbotXL.</p>
                <p class="small">
                    Left: forward/backward (vertical axis)<br/>
                    Right: rotation (horizontal axis)
                </p>
            </div>

            <div id="status">Ready. Keyboard Mode. Press a key.</div>
            <div class="log" id="log"></div>
        </div>

        <div id="joysticks" class="joysticks hidden">
            <div class="joystick-container" id="leftJoy">
                <div class="joystick-knob"></div>
            </div>
            <div class="joystick-container" id="rightJoy">
                <div class="joystick-knob"></div>
            </div>
        </div>

        <script>
            const statusEl = document.getElementById('status');
            const logEl = document.getElementById('log');
            const btnKeyboard = document.getElementById('btnKeyboard');
            const btnAnalog = document.getElementById('btnAnalog');
            const keyboardHelp = document.getElementById('keyboardHelp');
            const analogHelp = document.getElementById('analogHelp');
            const joysticksEl = document.getElementById('joysticks');

            let mode = "keyboard";  // "keyboard" | "analog"

            const allowedKeys = new Set([
                "i","o","j","l","u",",",".","m",
                "O","I","J","L","U","<",">","M",
                "t","b",
                "q","z","w","x","e","c",
                "v"
            ]);

            function log(msg) {{
                const line = document.createElement("div");
                const time = new Date().toLocaleTimeString();
                line.textContent = "[" + time + "] " + msg;
                logEl.prepend(line);
            }}

            // --- Keyboard - Joypad toggle ---

            function setMode(newMode) {{
                if (newMode === mode) return;
                mode = newMode;

                if (mode === "keyboard") {{
                    btnKeyboard.classList.add("active");
                    btnAnalog.classList.remove("active");
                    keyboardHelp.classList.remove("hidden");
                    analogHelp.classList.add("hidden");
                    joysticksEl.classList.add("hidden");
                    statusEl.textContent = "Ready. Keyboard Mode.";
                    sendAnalog(0.0, 0.0); // safety reset for the analogs
                }} else {{
                    btnKeyboard.classList.remove("active");
                    btnAnalog.classList.add("active");
                    keyboardHelp.classList.add("hidden");
                    analogHelp.classList.remove("hidden");
                    joysticksEl.classList.remove("hidden");
                    statusEl.textContent = "Ready. Joypad mode.";
                }}
            }}

            btnKeyboard.addEventListener("click", () => setMode("keyboard"));
            btnAnalog.addEventListener("click", () => setMode("analog"));

            // --- Keyboard handler ---

            document.addEventListener("keydown", function(ev) {{
                if (mode !== "keyboard") return;

                const key = ev.key;

                if (!allowedKeys.has(key)) {{
                    return;
                }}

                statusEl.textContent = "Sending key '" + key + "' to backend...";
                statusEl.className = "";

                fetch("/teleop/key", {{
                    method: "POST",
                    headers: {{
                        "Content-Type": "application/json"
                    }},
                    body: JSON.stringify({{ key: key }})
                }})
                .then(res => {{
                    if (!res.ok) {{
                        return res.json().then(err => {{
                            throw new Error(err.detail || ("HTTP " + res.status));
                        }});
                    }}
                    return res.json();
                }})
                .then(data => {{
                    statusEl.textContent = "RPC OK for key '" + key + "'";
                    statusEl.className = "ok";
                    log("key='" + key + "' → " + JSON.stringify(data.rpc_result));
                }})
                .catch(err => {{
                    statusEl.textContent = "Error for key '" + key + "': " + err.message;
                    statusEl.className = "err";
                    log("ERROR key='" + key + "': " + err.message);
                }});
            }});

            // Analogs

            let currentLX = 0.0;
            let currentRX = 0.0;

            function sendAnalog(lx, rx) {{
                currentLX = lx;
                currentRX = rx;

                if (mode !== "analog") return;

                statusEl.textContent = "Sending analog lx=" + lx.toFixed(2) + " rx=" + rx.toFixed(2);
                statusEl.className = "";

                fetch("/teleop/analog", {{
                    method: "POST",
                    headers: {{
                        "Content-Type": "application/json"
                    }},
                    body: JSON.stringify({{
                        lx: lx,
                        rx: rx,
                        turbo: false
                    }})
                }})
                .then(res => {{
                    if (!res.ok) {{
                        return res.json().then(err => {{
                            throw new Error(err.detail || ("HTTP " + res.status));
                        }});
                    }}
                    return res.json();
                }})
                .then(data => {{
                    statusEl.textContent = "RPC OK (analog)";
                    statusEl.className = "ok";
                    log("analog lx=" + lx.toFixed(2) + " rx=" + rx.toFixed(2) +
                        " → " + JSON.stringify(data.rpc_result));
                }})
                .catch(err => {{
                    statusEl.textContent = "Errore analog: " + err.message;
                    statusEl.className = "err";
                    log("ERROR analog: " + err.message);
                }});
            }}

            function setupJoystick(containerId, onChange) {{
                const container = document.getElementById(containerId);
                const knob = container.querySelector(".joystick-knob");

                let active = false;

                function resetKnob() {{
                    knob.style.left = "50%";
                    knob.style.top = "50%";
                }}

                function handlePointerDown(ev) {{
                    if (mode !== "analog") return;
                    active = true;
                    container.setPointerCapture(ev.pointerId);
                    updateFromEvent(ev);
                }}

                function handlePointerMove(ev) {{
                    if (!active) return;
                    updateFromEvent(ev);
                }}

                function handlePointerUp(ev) {{
                    if (!active) return;
                    active = false;
                    container.releasePointerCapture(ev.pointerId);
                    resetKnob();
                    onChange(0, 0, false);
                }}

                function updateFromEvent(ev) {{
                    const rect = container.getBoundingClientRect();
                    const cx = rect.left + rect.width / 2;
                    const cy = rect.top + rect.height / 2;
                    const maxR = rect.width / 2;

                    const dx = ev.clientX - cx;
                    const dy = ev.clientY - cy;

                    const r = Math.sqrt(dx*dx + dy*dy);
                    let ndx = dx;
                    let ndy = dy;
                    if (r > maxR) {{
                        const s = maxR / r;
                        ndx *= s;
                        ndy *= s;
                    }}

                    const px = 50 + (ndx / maxR) * 50;
                    const py = 50 + (ndy / maxR) * 50;
                    knob.style.left = px + "%";
                    knob.style.top = py + "%";

                    // normalizza in [-1,1]
                    const normX = ndx / maxR;
                    const normY = ndy / maxR; 

                    onChange(normX, normY, true);
                }}

                container.addEventListener("pointerdown", handlePointerDown);
                container.addEventListener("pointermove", handlePointerMove);
                container.addEventListener("pointerup", handlePointerUp);
                container.addEventListener("pointercancel", handlePointerUp);
            }}

            setupJoystick("leftJoy", (nx, ny, isActive) => {{
                if (!isActive) {{
                    sendAnalog(0.0, currentRX);
                    return;
                }}
                // ny > 0 = giù; per convenzione vogliamo ly positivo avanti (su)
                const ly = -ny; // inverti
                const lx = Math.max(-1, Math.min(1, ly));
                sendAnalog(lx, currentRX);
            }});

            // Joystick destro: usa solo asse orizzontale → angular.z
            setupJoystick("rightJoy", (nx, ny, isActive) => {{
                if (!isActive) {{
                    // rilasciato: solo rx = 0, lx invariato
                    sendAnalog(currentLX, 0.0);
                    return;
                }}
                const rx = Math.max(-1, Math.min(1, -nx));
                sendAnalog(currentLX, rx);
            }});
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=4053)
