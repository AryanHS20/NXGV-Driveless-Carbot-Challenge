function updateFlow(currentState) {
  const activeIdx = PRIORITY_ORDER.indexOf(currentState);
  
  PRIORITY_ORDER.forEach((s, i) => {
    const el = document.getElementById('flow_' + s);
    if (!el) return;
    
    el.classList.remove('active', 'done');
    // If it's the active state, highlight it
    if (i === activeIdx) {
      el.classList.add('active');
    } 
    // If it's a lower priority state than current, mark it as "done" (overridden)
    else if (i < activeIdx) {
      el.classList.add('done');
    }
  });

  // Color arrows. Arrow i is between node i and i+1.
  const arrows = document.querySelectorAll('.flow-arrow');
  arrows.forEach((a, i) => {
    // If the active state is higher than i, the arrow is lit up implying flow of priority
    a.classList.toggle('passed', i < activeIdx);
  });
}

