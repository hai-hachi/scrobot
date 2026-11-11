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
  "li-rows", "li-cols", "li-reach_x", "li-reach_y", "li-height",
  "light-h", "light-s", "light-v", "li-intensity",
  "li-constant", "li-quadratic",
  "venue-clearance_x", "venue-clearance_y", "court-h", "poles-v",
]) {
  assert(registry.has(id), "Missing dynamically rendered slider " + id);
}
const lights = vm.runInContext("lightPayload()", context);
assert.equal(lights.rows, 2);
assert.equal(lights.cols, 4);
assert.equal(lights.shadow_count, 2);
assert.equal(lights.kind, "spot");
assert.equal(lights.reach_x, 4.5);
assert.equal(lights.reach_y, 4.6);
assert.deepEqual(Array.from(lights.hsv), [55, 3, 95]);
assert.equal(lights.intensity, 1);
assert(!Object.hasOwn(lights, "temperature"));
assert(!Object.hasOwn(lights, "brightness"));

const colors = vm.runInContext("materialsPayload()", context);
assert.deepEqual(Array.from(colors.court), [147, 84, 35]);
assert(!Object.hasOwn(colors, "pole_diameter"));
const venue = vm.runInContext("venuePayload()", context);
assert.equal(venue.clearance_x, 2.3);
assert.equal(venue.clearance_y, 2.4);

// Opening a saved preset must initialize the controls from the backend,
// rather than misleadingly displaying the default light array and colors.
vm.runInContext(
  "syncControls({lights:{rows:4,cols:2,reach_x:5,reach_y:6,hsv:[270,35,80],intensity:2.0,kind:'point',shadows:false},"+
  "colors:{court:[100,50,70]},"+
  "venue:{clearance_x:3.5,clearance_y:1.75},"+
  "robot:{x:1.5,y:0,yaw:90},"+

  "shuttles:{count:33,seed:12345,density:'net'}})",
  context
);
assert.equal(registry.get("li-rows").value, "4");
assert.equal(registry.get("li-cols").value, "2");
assert.equal(registry.get("light-kind").value, "point");
assert.equal(registry.get("light-shadows").checked, false);
assert.equal(registry.get("li-reach_x").value, "5");
assert.equal(registry.get("light-h").value, "270");
assert.equal(registry.get("light-s").value, "35");
assert.equal(registry.get("light-v").value, "80");
assert.equal(registry.get("li-intensity").value, "2");
assert.equal(registry.get("venue-clearance_x").value, "3.5");
assert.equal(registry.get("venue-clearance_y").value, "1.75");
assert(!registry.has("pole-dia"));
assert(registry.has("light-h"));
assert.equal(registry.get("shuttle-count").value, "33");
assert(!registry.has("free-z"));
assert(!registry.has("free-yaw"));
assert(!registry.has("move-free"));

// Exercise an event after setup and verify swatch appearance changes.
const hue = registry.get("court-h");
hue.value = "275";
hue.dispatchEvent({type: "input"});
assert.match(registry.get("sw-court").style.background, /275/);

// The Gazebo Orbit follow toggle and clearance apply must have live actions.
assert(registry.has("follow-orbit"));
assert(registry.has("copy-orbit"));
assert(registry.has("apply-venue"));
assert.match(html, /\/api\/orbit/);
assert.match(html, /\/api\/venue/);
assert(registry.has("update-lab-config"));
assert.match(html, /\/api\/config/);
assert(registry.has("preview-mode"), "Missing preview quality selector");
assert(registry.has("shadow-preview"), "Missing shadow-free preview toggle");
assert(registry.has("show-light-frames"), "Missing Gazebo light frame checkbox");
assert(registry.has("light-y-layout"), "Missing applied Y-spread diagnostics");
assert(registry.has("light-frame-status"), "Missing overlay readiness status");
assert.match(html, /\/api\/light_frames/);
assert(registry.has("light-update-timing"), "Missing lighting latency metric");
assert(registry.has("apply-render-budget"), "Missing real Gazebo rendering budget control");
assert(registry.has("download-benchmark"), "Missing performance samples export");
assert(registry.has("robot-visual-mode"), "Missing robot visual simplification selector");
assert(registry.has("inspection-profile"), "Missing Gazebo inspection sensor profile selector");
assert(registry.has("enable-depth-camera"), "Missing depth sensor toggle");
assert.match(html, /\/api\/render_budget/);
assert(registry.has("material-update-timing"), "Missing material latency metric");
assert.match(html, /\/api\/preview/);
assert.match(html, /frameBusy\[key\]/);
assert.match(html, /queueApply\("lights",lightPayload\(\)\)/);
assert(registry.has("publish-simulation"), "Missing main simulation publish button");
assert.match(html, /\/api\/publish/);

// The court click/robot camera pose selection must be wired.
assert((registry.get("courtmap").listeners.click || []).length >= 1);
assert.equal((registry.get("feed-free").listeners.pointerup || []).length, 0);

console.log("Lighting lab browser initialization, HSV lights, orbit sync, YAML action: OK");
