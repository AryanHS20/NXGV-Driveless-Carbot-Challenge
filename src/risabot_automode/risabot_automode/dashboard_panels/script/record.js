function rpCmd(action) {
  fetch('/api/record_playback', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action: action})
  }).catch(() => {});
}

