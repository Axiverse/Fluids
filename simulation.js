/*
Fluid Simulation
https://mikeash.com/pyblog/fluid-simulation-for-dummies.html


*/

Number.prototype.mod = function(n) {
    return ((this%n)+n)%n;
};

const clamp = (value, min, max) => Math.max(Math.min(value, max), min);
const lerp = (a, b, scale) => (a * (1 - scale)) + (b * scale);
const wrap = (a, max) => {
	while (a < 0) a += max;
	while (a >= max) a -= max;
	return a;
}
const bilerp = (sampler, x, y) => {
	const x0 = ~~x;
	const y0 = ~~y;
	const x1 = x0 + 1;
	const y1 = y0 + 1;
    const p00 = sampler(x0, y0);
    const p01 = sampler(x0, y1);
    const p10 = sampler(x1, y0);
    const p11 = sampler(x1, y1);
    return lerp(
    	lerp(p00, p10, x - x0),
    	lerp(p01, p11, x - x0),
    	y - y0);
}
const nearestSampler = (array, width, height) => (x, y) => {
	return array[wrap(~~y, height) + wrap(~~x, width) * width];
}
const nearestSamplerComponent = (array, width, height, component, components) => (x, y) => {
	return array[(wrap(~~y, height) + wrap(~~x, width) * width) * components + component];
}
const bilerpSampler = (sampler) => (x, y) => bilerp(sampler, x, y);
const bilerpRescaleSampler = (sampler, width, height, samplerWidth, samplerHeight) => {
	const sx = width / samplerWidth;
	const sy = height / samplerHeight;
	return (x, y) => bilerp(sampler, wrap(x * sx, width), wrap(y * sy, height));
}
const advect = (svx, svy, src, dest, t) => {
	const r = bilerpSampler(nearestSamplerComponent(src.data, canvasWidth, canvasHeight, 0, 4));
	const g = bilerpSampler(nearestSamplerComponent(src.data, canvasWidth, canvasHeight, 1, 4));
	const b = bilerpSampler(nearestSamplerComponent(src.data, canvasWidth, canvasHeight, 2, 4));
	const a = nearestSamplerComponent(src.data, canvasWidth, canvasHeight, 3, 4);
	for (let x = 0; x < canvasWidth; ++x) {
		for (let y = 0; y < canvasHeight; ++y) {
			const vx = -svx(x, y) * t;
			const vy = -svy(x, y) * t;
			const i = (y + x * canvasWidth) * 4;
			dest.data[i + 0] = r(x + vx, y + vy);
			dest.data[i + 1] = g(x + vx, y + vy);
			dest.data[i + 2] = b(x + vx, y + vy);
			dest.data[i + 3] = 255;
		}	
	}
}


let canvas = null;
let ctx = null;

const canvasWidth = 256;
const canvasHeight = 256;
const gridWidth = 16;
const gridHeight = 16;

let srcBuffer = null;
let destBuffer = null;

let vx0 = new Float32Array(gridWidth * gridHeight);
let vy0 = new Float32Array(gridWidth * gridHeight);
let vx = new Float32Array(gridWidth * gridHeight);
let vy = new Float32Array(gridWidth * gridHeight);
let s = new Float32Array(gridWidth * gridHeight);
let density = new Float32Array(gridWidth * gridHeight);
const gridIndex = (x, y) => y.mod(gridHeight) + x.mod(gridWidth) * gridWidth;

const drawCheckerboard = () => {
	const width = 16;
	const height = 16;

	let column = false;

	ctx.fillStyle = '#fff';
	ctx.fillRect(0, 0, canvasWidth, canvasHeight);
	ctx.fillStyle = '#000';

	for (var x = 0; x < canvasWidth; x += width) {
		let row = column;
		column = !column;

		for (var y = 0; y < canvasHeight; y += height) {
			if (row) {
				ctx.fillRect(x, y, width, height);
			}
			//console.log(row);
			row = !row;
		}
	}
}

// Solves iteratively a constraint
const linearSolver = (v, vInitial, a, c, iterations) => {
	for (var i = 0; i < iterations; i++) {
		for (var x = 0; x < gridWidth; x++) {
			for (var y = 0; y < gridHeight; y++) {
				const initial = vInitial[gridIndex(x, y)];
				const neighbors =
					v[gridIndex(x + 1, y)] +
					v[gridIndex(x - 1, y)] +
					v[gridIndex(x, y + 1)] +
					v[gridIndex(x, y - 1)];

				v[gridIndex(x, y)] = (initial + neighbors * a) / c;
			}
		}
	}
}

const diffuse = (v, vInitial, diffuse, dt, iterations) => {
	const a = dt * diffuse * gridWidth * gridHeight;
	linearSolver(v, vInitial, a, 1 + 4 * a, iterations);
}

const project = (vX, vY, pressure, divergence, iterations) => {

}

const start = () => {
	srcBuffer = ctx.getImageData(0, 0, canvasWidth, canvasHeight);
	destBuffer = ctx.createImageData(canvasWidth, canvasHeight);

}

const step = () => {

	const dt = 0.1;

	diffuse(vx0, vx, dt, 4);
	diffuse(vy0, vy, dt, 4);

	const svx = bilerpRescaleSampler(nearestSampler(vx, gridWidth, gridHeight), gridWidth, gridHeight, canvasWidth, canvasHeight);
	const svy = bilerpRescaleSampler(nearestSampler(vy, gridWidth, gridHeight), gridWidth, gridHeight, canvasWidth, canvasHeight);

	advect(svx, svy, srcBuffer, destBuffer, dt);
	ctx.putImageData(destBuffer, 0, 0);

	const swapBuffer = destBuffer;
	destBuffer = srcBuffer;
	srcBuffer = swapBuffer;

	drawFlow(vx, vy);


	const swapX = vx;
	vx = vx0;
	vx0 = swapX;

	const swapY = vy;
	vy = vy0;
	vy0 = swapY;

	//console.log('+');
	window.requestAnimationFrame(step);
}


const drawMarker = (x, y, dx, dy) => {
	const s = 0.2;
	ctx.fillStyle = '#f00';
	ctx.beginPath();
	ctx.moveTo(x + dx, y + dy);
	ctx.lineTo(x + dx * s, y - dy * s);
	ctx.lineTo(x - dx * s, y + dy * s);
	ctx.closePath();
	ctx.fill();
}

const drawFlow = (vx, vy) => {
	const spaceWidth = canvasWidth / gridWidth;
	const spaceHeight = canvasHeight / gridHeight;

	const sx = spaceWidth / 2;
	const sy = spaceHeight / 2;

	for (let x = 0; x < gridWidth; ++x) {
		for (let y = 0; y < gridHeight; ++y) {
			const dx = vx[y + x * gridWidth];
			const dy = vy[y + x * gridWidth];
			drawMarker(sx + x * spaceWidth, sy + y * spaceHeight, dx, dy);
		}
	}
}

const mapGrid = (grid, map) => {
	for (let x = 0; x < gridWidth; ++x) {
		for (let y = 0; y < gridHeight; ++y) {
			const u = (x + 0.5) / gridWidth;
			const v = (y + 0.5) / gridHeight;
			grid[y + x * gridWidth] = map(u, v);
		}
	}
}
const load = () => {
	canvas = document.getElementById('canvas');
	ctx = canvas.getContext('2d');	

	drawCheckerboard();
	mapGrid(vx, (u, v) => 10);
	mapGrid(vy, (u, v) => 10);
	start();

	window.requestAnimationFrame(step);
}



window.onload = load;