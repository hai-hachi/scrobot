#!/usr/bin/env node
"use strict";
/* Small dependency-free DOM stub for bootstrapping the embedded dashboard.
 * This specifically catches JavaScript runtime initialization errors that
 * node --check (syntax-only) cannot detect.
 */
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");

const html = fs.readFileSync(path.join(__dirname, "index.html"), "utf8");
const match = html.match(/<script>([\s\S]*?)<\/script>/);
assert(match, "Missing inline application script");

const registry = new Map();
class Element {
  constructor(id = "") {
    this._id = "";
    this.value = "";
    this.min = "";
    this.max = "";
    this.checked = false;
    this.style = {};
    this.listeners = {};
    this.classList = {add() {}, remove() {}};
    this.children = [];
    this.textContent = "";
    this.complete = false;
    this.naturalWidth = 0;
    this.tagName = "DIV";
    if (id) this.id = id;
  }
  set id(value) {
    this._id = value;
    if (value) registry.set(value, this);
  }
  get id() {return this._id;}
  appendChild(child) {this.children.push(child); return child;}
  addEventListener(event, fn) {
    (this.listeners[event] ||= []).push(fn);
  }
  dispatchEvent(event) {
    for (const fn of this.listeners[event.type] || []) fn(event);
    return true;
  }
  setAttribute(key, val) {this[key] = String(val);}
  getBoundingClientRect() {return {left: 0, top: 0, width: 360, height: 170};}
  setPointerCapture() {}
  getContext() {
    return {
      clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {}, stroke() {},
      drawImage() {},
    };
  }
}

// Register actual static HTML elements, retaining their value/checked fields.
const tags = /<([a-z][\w-]*)\b([^>]*\bid="[^"]+"[^>]*)>/gi;
for (const [, tag, attrs] of html.matchAll(tags)) {
  const id = attrs.match(/\bid="([^"]+)"/)?.[1];
  if (!id) continue;
  const elem = new Element(id);
  elem.tagName = tag.toUpperCase();
  elem.value = attrs.match(/\bvalue="([^"]*)"/)?.[1] || "";
  elem.checked = /\bchecked\b/.test(attrs);
}
// Browsers select the first <option> automatically when <select>
// has no explicit value attribute. Reproduce that behavior in this stub.
registry.get("light-kind").value = "spot";
registry.get("shuttle-density").value = "uniform";
const document = {
  getElementById(id) {return registry.get(id) || null;},
  createElement(tag) {const elem = new Element();elem.tagName=tag.toUpperCase();return elem;},
  activeElement: {tagName: "BODY"},
};
const win = {addEventListener() {}, lastErr: ""};
const context = vm.createContext({
  document, window: win,
  Event: class {constructor(type) {this.type = type;}},
  setInterval() {},
  setTimeout() {},
  clearTimeout() {},
  fetch() {return Promise.reject(new Error("expected: no server in DOM smoke test"));},
  Math, Number, Object, Array, Date, JSON, console, URL,
});
vm.runInContext(match[1], context, {timeout: 4000});

// Initialization must render every control without referencing an absent id.
for (const id of [
  "li-rows", "li-cols", "li-span_x", "li-span_y", "li-height",
  "li-intensity", "li-constant", "li-quadratic",
  "light-h", "light-s", "light-v", "court-h", "poles-v",
]) {
  assert(registry.has(id), "Missing dynamically rendered slider " + id);
}
const lights = vm.runInContext("lightPayload()", context);
assert.equal(lights.rows, 2);
assert.equal(lights.cols, 3);
assert.equal(lights.kind, "spot");
assert.deepEqual(Array.from(lights.hsv), [55, 3, 95]);

const colors = vm.runInContext("materialsPayload()", context);
assert.deepEqual(Array.from(colors.court), [147, 84, 35]);
assert.equal(colors.pole_diameter, 0.05);

// Opening a saved preset must initialize the controls from the backend,
// rather than misleadingly displaying the default light array and colors.
vm.runInContext(
  "syncControls({lights:{rows:4,cols:2,hsv:[25,70,80],kind:'point',shadows:false},"+
  "colors:{court:[100,50,70],pole_diameter:0.09},"+
  "robot:{x:1.5,y:0,yaw:90},"+
  "free:{x:4,y:1,z:9,yaw:75,pitch:40},"+
  "shuttles:{count:33,seed:12345,density:'net'}})",
  context
);
assert.equal(registry.get("li-rows").value, "4");
assert.equal(registry.get("li-cols").value, "2");
assert.equal(registry.get("light-kind").value, "point");
assert.equal(registry.get("light-shadows").checked, false);
assert.equal(registry.get("pole-dia").value, "0.09");
assert.equal(registry.get("shuttle-count").value, "33");
assert.equal(registry.get("free-z").value, "9");

// Exercise an event after setup and verify swatch appearance changes.
const hue = registry.get("court-h");
hue.value = "275";
hue.dispatchEvent({type: "input"});
assert.match(registry.get("sw-court").style.background, /275/);

// The court click/robot camera pose selection must be wired.
assert((registry.get("courtmap").listeners.click || []).length >= 1);
assert((registry.get("feed-free").listeners.pointerup || []).length >= 1);

console.log("Lighting lab browser initialization, HSV sliders, camera controls: OK");
