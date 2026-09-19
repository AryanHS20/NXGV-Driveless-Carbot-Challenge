import React from 'react';

/** Tesla-style takeover pill; silent when healthy. */
export default function Banner({data}) {
  const stale = [...(data.stale_streams || []), ...(data.health_stale || [])];
  let text = 'Systems nominal';
  let kind = '';
  if (data.state === 'EMERGENCY_STOP') {
    text = 'EMERGENCY STOP';
    kind = 'alert';
  } else if (stale.length > 0) {
    text = `Take over — ${stale.slice(0, 2).join(', ')} stale`;
    kind = 'alert';
  } else if (data.lane_lost === true) {
    text = 'Lane lost — be ready to take over';
    kind = 'warn';
  }
  return <div className={`banner ${kind}`}>{text}</div>;
}
