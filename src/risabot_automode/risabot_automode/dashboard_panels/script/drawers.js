function toggleCtrlDrawer() {
  document.getElementById('ctrlDrawer').classList.toggle('open');
}

function toggleParamDrawer() {
  document.getElementById('paramDrawer').classList.toggle('open');
  document.querySelector('.layout').classList.toggle('drawer-open');
}

