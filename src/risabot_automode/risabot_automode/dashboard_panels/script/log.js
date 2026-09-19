function addLogEntry(text) {
  const now = new Date();
  const ts = now.toLocaleTimeString('en-GB');
  eventLog.unshift({time: ts, text: text});
  if (eventLog.length > 30) eventLog.pop();
  const container = document.getElementById('logScroll');
  container.innerHTML = eventLog.map(e =>
    `<div class="log-entry"><span class="log-time">${e.time}</span><span class="log-event">${e.text}</span></div>`
  ).join('');
}

