/** Render smoke test: App mounts and paints mock telemetry (no robot needed). */
// @vitest-environment jsdom
import { strict as assert } from 'node:assert';
import React from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, test, vi } from 'vitest';
import App from '../src/App.jsx';

vi.stubGlobal('requestAnimationFrame', (cb) => setTimeout(() => cb(Date.now()), 16));

afterEach(() => {
  document.body.innerHTML = '<div id="root"></div>';
});

test('app renders cluster, scene, banner and plots from mock data', async () => {
  document.body.innerHTML = '<div id="root"></div>';
  const root = createRoot(document.getElementById('root'));
  root.render(<App />);
  await new Promise(resolve => setTimeout(resolve, 1500));
  const text = document.body.textContent;
  assert.match(text, /M\/S/);
  assert.match(text, /GEAR/);
  assert.match(text, /MANUAL|AUTO/);
  assert.ok(document.querySelector('svg.scene'));
  assert.equal(document.querySelectorAll('svg.spark').length, 2);
  assert.match(text, /Systems nominal|Take over|Lane lost|EMERGENCY/);
  root.unmount();
}, 10000);
