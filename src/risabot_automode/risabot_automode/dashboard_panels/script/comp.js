function sendCompCmd(cmd) {
  fetch('/api/reset_competition', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({command: cmd})
  })
  .then(r => r.json())
  .then(data => {
    if(data.ok) {
      addLogEntry(`Sent command: <span class="log-val">${cmd}</span>`);
      update();
    } else {
      addLogEntry(`⚠️ Error: ${data.error}`);
    }
  })
  .catch(err => {
    addLogEntry(`⚠️ Fetch Error: ${err}`);
  });
}

