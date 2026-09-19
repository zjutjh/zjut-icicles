const registerNames = ["R0", "R1", "R2", "R3"];
const stateNames = ["PC", "IR", "AR", "SP", "FZ", "FC", "EI", "INTR", "uAR"];
const logView = document.querySelector("#logView");
const inBitsInput = document.querySelector("#inBits");
let running = false;
let runTimer = null;
let inputSyncTimer = null;
let memoryMode = "main";
let latestSnapshot = null;
let memoryBackendWarningShown = false;
let requestEpoch = 0;

function field(name) {
  const node = document.createElement("div");
  node.className = "field";
  node.innerHTML = `<span>${name}</span><strong id="field-${name}">00</strong>`;
  return node;
}

function initFields() {
  const registerGrid = document.querySelector("#registerGrid");
  const stateGrid = document.querySelector("#stateGrid");
  registerNames.forEach((name) => registerGrid.appendChild(field(name)));
  stateNames.forEach((name) => stateGrid.appendChild(field(name)));
  const switchBank = document.querySelector("#switchBank");
  for (let bit = 7; bit >= 0; bit -= 1) {
    const label = document.createElement("label");
    label.className = "bit-switch";
    label.innerHTML = `<input type="checkbox" data-bit="${bit}"><span>${bit}</span>`;
    switchBank.appendChild(label);
  }
  initMemoryTable();
}

function hex(value) {
  return Number(value || 0).toString(16).toUpperCase().padStart(2, "0").slice(-2);
}

function hex6(value) {
  return Number(value || 0).toString(16).toUpperCase().padStart(6, "0").slice(-6);
}

function bit(value) {
  return Number(value || 0).toString();
}

async function api(path, body = {}) {
  const response = await fetch(path, {
    method: path === "/api/state" ? "GET" : "POST",
    headers: path === "/api/state" ? {} : { "Content-Type": "application/json" },
    body: path === "/api/state" ? undefined : JSON.stringify(body),
  });
  const data = await response.json();
  if (!response.ok || data.error) {
    throw new Error(data.error || `HTTP ${response.status}`);
  }
  return data;
}

function render(data) {
  latestSnapshot = data;
  const state = data.state || {};
  registerNames.forEach((name) => {
    document.querySelector(`#field-${name}`).textContent = hex(state[name]);
  });
  stateNames.forEach((name) => {
    const target = document.querySelector(`#field-${name}`);
    target.textContent = ["FZ", "FC", "EI", "INTR"].includes(name) ? bit(state[name]) : hex(state[name]);
  });
  document.querySelector("#outValue").textContent = hex(state.OUT);
  document.querySelector("#instructionCount").textContent = String(data.instructions || 0);
  document.querySelector("#microStepCount").textContent = String(data.microSteps || 0);
  document.querySelector("#haltedState").textContent = data.halted ? "HALT" : running ? "RUN" : "READY";
  document.querySelector("#loadedName").textContent = data.loadedName || "未加载文件";
  renderMemory(data);
  appendLogs(data.logs || []);
  document.querySelector("#runBtn").textContent = running ? "暂停运行" : "连续运行";
}

function setInputControls(bits) {
  inBitsInput.value = bits;
  updateSwitchesFromInput();
}

function getInputBits() {
  return inBitsInput.value.trim();
}

function validBits(bits) {
  return /^[01]{8}$/.test(bits);
}

async function syncInputNow({ quiet = false } = {}) {
  const bits = getInputBits();
  if (!validBits(bits)) {
    throw new Error("IN 必须是 8 位二进制数");
  }
  const data = await api("/api/input", { bits });
  if (quiet) {
    data.logs = [];
  }
  return data;
}

function syncInputSoon() {
  if (inputSyncTimer) {
    clearTimeout(inputSyncTimer);
  }
  inputSyncTimer = setTimeout(() => {
    if (!validBits(getInputBits())) {
      return;
    }
    guarded(() => syncInputNow({ quiet: true }));
  }, 120);
}

function appendLogs(lines) {
  if (!lines.length) {
    return;
  }
  const text = lines.join("\n") + "\n";
  logView.textContent += text;
  if (logView.textContent.length > 120000) {
    logView.textContent = logView.textContent.slice(-90000);
  }
  logView.scrollTop = logView.scrollHeight;
}

function initMemoryTable() {
  const table = document.querySelector("#memoryTable");
  for (let address = 0; address <= 0xff; address += 1) {
    const row = document.createElement("div");
    row.className = "memory-row";
    row.dataset.address = String(address);
    row.innerHTML = `<span>${hex(address)}</span><strong>00</strong>`;
    table.appendChild(row);
  }
}

function renderMemory(data) {
  const source = memoryMode === "micro" ? data.microMemory : data.mainMemory;
  const values = Array.isArray(source) ? source : null;
  document.querySelectorAll("#memoryTable .memory-row").forEach((row) => {
    const address = Number(row.dataset.address);
    const valueNode = row.querySelector("strong");
    if (!values) {
      row.classList.add("empty");
      valueNode.textContent = "--";
      return;
    }
    const value = values[address];
    const emptyMicroCell = memoryMode === "micro" && value == null;
    row.classList.toggle("empty", emptyMicroCell);
    valueNode.textContent = memoryMode === "micro" ? (emptyMicroCell ? "------" : hex6(value)) : hex(value);
  });
  if (!values) {
    if (!memoryBackendWarningShown) {
      appendLogs(["WARN: 当前后端未返回存储器数据，请重启模拟器服务后刷新页面"]);
      memoryBackendWarningShown = true;
    }
  } else {
    memoryBackendWarningShown = false;
  }
  document.querySelector("#showMainMemoryBtn").classList.toggle("active", memoryMode === "main");
  document.querySelector("#showMicroMemoryBtn").classList.toggle("active", memoryMode === "micro");
}

function setMemoryMode(mode) {
  memoryMode = mode;
  if (latestSnapshot) {
    renderMemory(latestSnapshot);
  }
}

function invalidatePendingRequests() {
  requestEpoch += 1;
  return requestEpoch;
}

async function guarded(action, epoch = requestEpoch) {
  try {
    const data = await action();
    if (epoch !== requestEpoch) {
      return null;
    }
    render(data);
    return data;
  } catch (error) {
    if (epoch === requestEpoch) {
      appendLogs([`ERROR: ${error.message}`]);
      stopRun();
    }
    return null;
  }
}

function stopRun() {
  running = false;
  if (runTimer) {
    clearTimeout(runTimer);
    runTimer = null;
  }
  document.querySelector("#runBtn").textContent = "连续运行";
}

function scheduleRun(epoch = requestEpoch) {
  if (!running || epoch !== requestEpoch) {
    return;
  }
  runTimer = setTimeout(async () => {
    const data = await guarded(async () => {
      await syncInputNow({ quiet: true });
      return api("/api/step", { count: 1, logLimit: 8 });
    }, epoch);
    if (epoch !== requestEpoch) {
      return;
    }
    if (!data || data.halted) {
      stopRun();
      return;
    }
    scheduleRun(epoch);
  }, 80);
}

function wireEvents() {
  document.querySelector("#openFileBtn").addEventListener("click", () => {
    document.querySelector("#fileInput").click();
  });

  document.querySelector("#fileInput").addEventListener("change", async (event) => {
    const file = event.target.files[0];
    if (!file) {
      return;
    }
    const text = await file.text();
    stopRun();
    const epoch = invalidatePendingRequests();
    logView.textContent = "";
    await guarded(() => api("/api/load", { filename: file.name, text }), epoch);
  });

  document.querySelector("#loadExampleBtn").addEventListener("click", async () => {
    stopRun();
    const epoch = invalidatePendingRequests();
    logView.textContent = "";
    await guarded(() => api("/api/load-example"), epoch);
  });

  document.querySelector("#runBtn").addEventListener("click", () => {
    if (running) {
      stopRun();
      const epoch = invalidatePendingRequests();
      guarded(() => api("/api/state"), epoch);
      return;
    }
    const epoch = invalidatePendingRequests();
    running = true;
    appendLogs(["连续运行开始"]);
    guarded(async () => {
      await syncInputNow({ quiet: true });
      return api("/api/state");
    }, epoch).then((data) => {
      if (data) {
        scheduleRun(epoch);
      }
    });
  });

  document.querySelector("#stepBtn").addEventListener("click", async () => {
    stopRun();
    const epoch = invalidatePendingRequests();
    await guarded(async () => {
      await syncInputNow({ quiet: true });
      return api("/api/step", { count: 1, logLimit: null });
    }, epoch);
  });

  document.querySelector("#resetBtn").addEventListener("click", async () => {
    stopRun();
    const epoch = invalidatePendingRequests();
    await guarded(() => api("/api/reset"), epoch);
  });

  document.querySelector("#clearLogBtn").addEventListener("click", () => {
    logView.textContent = "";
  });

  document.querySelector("#showMainMemoryBtn").addEventListener("click", () => {
    setMemoryMode("main");
  });

  document.querySelector("#showMicroMemoryBtn").addEventListener("click", () => {
    setMemoryMode("micro");
  });

  inBitsInput.addEventListener("input", () => {
    const cleaned = getInputBits().replace(/[^01]/g, "").slice(0, 8);
    if (cleaned !== inBitsInput.value) {
      inBitsInput.value = cleaned;
    }
    updateSwitchesFromInput();
    syncInputSoon();
  });

  document.querySelectorAll("#switchBank input").forEach((input) => {
    input.addEventListener("change", () => {
      updateInputFromSwitches();
      syncInputSoon();
    });
  });

  document.querySelector("#interruptBtn").addEventListener("click", async () => {
    await guarded(async () => {
      await syncInputNow({ quiet: true });
      return api("/api/interrupt");
    });
  });
}

function updateSwitchesFromInput() {
  const bits = getInputBits().padEnd(8, "0").slice(0, 8);
  document.querySelectorAll("#switchBank input").forEach((input) => {
    const bit = Number(input.dataset.bit);
    input.checked = bits[7 - bit] === "1";
  });
}

function updateInputFromSwitches() {
  const bits = Array.from(document.querySelectorAll("#switchBank input"))
    .sort((left, right) => Number(right.dataset.bit) - Number(left.dataset.bit))
    .map((input) => (input.checked ? "1" : "0"))
    .join("");
  inBitsInput.value = bits;
}

initFields();
setInputControls("00000000");
wireEvents();
guarded(async () => {
  await syncInputNow({ quiet: true });
  return api("/api/state");
});
