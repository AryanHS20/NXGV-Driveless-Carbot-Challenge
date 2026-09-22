/** Render smoke test: App mounts and paints mock telemetry (no robot needed). */
// @vitest-environment jsdom
import { strict as assert } from 'node:assert';
import React from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, test, vi } from 'vitest';
import App from '../src/App.jsx';

vi.stubGlobal('requestAnimationFrame', (cb) => setTimeout(() => cb(Date.now()), 16));
vi.stubGlobal('cancelAnimationFrame', (id) => clearTimeout(id));

afterEach(() => {
  document.body.innerHTML = '<div id="root"></div>';
});

test('app renders cluster, canvas scene, banner and plots from mock data', async () => {
  document.body.innerHTML = '<div id="root"></div>';
  const root = createRoot(document.getElementById('root'));
  root.render(<App />);
  await new Promise(resolve => setTimeout(resolve, 1500));
  const text = document.body.textContent;
  assert.match(text, /M\/S/);
  assert.match(text, /MAX/);
  assert.match(text, /MANUAL|AUTO/);
  // Canvas scene instead of SVG
  assert.ok(document.querySelector('canvas.scene'), 'Expected canvas.scene element');
  assert.equal(document.querySelectorAll('svg.spark').length, 3, 'Expected 3 sparklines');
  assert.match(text, /Systems nominal|Take over|Lane lost|EMERGENCY/);
  root.unmount();
}, 10000);
