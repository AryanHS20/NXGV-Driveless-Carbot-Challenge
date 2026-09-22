import { chromium } from 'playwright';

const target = process.argv[2];
const out = process.argv[3];
const waitMs = Number(process.argv[4] || '3500');

const browser = await chromium.launch();
const page = await browser.newPage({viewport: {width: 480, height: 1000}});
await page.goto(target);
await page.waitForTimeout(waitMs);
await page.screenshot({path: out, fullPage: true});
await browser.close();
console.log('shot:', out);
