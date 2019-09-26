

// http://jamie-wong.com/2016/08/05/webgl-fluid-simulation/
// https://www.esimov.com/2014/01/navier-stokes-fluid-simulation-on-html5-canvas

const mod = (n, d) => ((n % d) + d) % d;
const lerp = (a, b, s) => (a * (1 - s)) + (b * s);
const clamp = (v, lo, hi) => Math.max(Math.min(v, hi), lo);

class Buffer {
	constructor(width, height, depth) {
		const size = width * height * depth;
		this.width = width;
		this.height = height;
		this.depth = depth;
		this.a = new Float32Array(size);
	}

	random(s, o) {
		const scale = (s || 1) * 2;
		const offset = o || 0;
		for (let i = 0; i < this.a.length; i++) {
			this.a[i] = (Math.random() - 0.5) * scale + offset;
		}
	}

	clear(v) {
		const value = v || 0;
		for (let i = 0; i < this.a.length; i++) {
			this.a[i] = value;
		}
	}
 
 	/** Bilinear scaled sampling where u, v, [0, 1). */
	sample(u, v, w) {
		const x = u * this.width;
		const y = v * this.height;
		const z = w * this.depth;

		const loX = Math.floor(x);
		const hiX = loX + 1;
		const loY = Math.floor(y);
		const hiY = loY + 1;
		const loZ = Math.floor(z);
		const hiZ = loZ + 1;

		const a00 = this.get(loX, loY, loZ);
		const a01 = this.get(loX, hiY, loZ);
		const a10 = this.get(hiX, loY, loZ);
		const a11 = this.get(hiX, hiY, loZ);

		const a = lerp(
			lerp(a00, a10, x - loX),
			lerp(a01, a11, x - loX),
			y - loY);

		const b00 = this.get(loX, loY, hiZ);
		const b01 = this.get(loX, hiY, hiZ);
		const b10 = this.get(hiX, loY, hiZ);
		const b11 = this.get(hiX, hiY, hiZ);

		const b = lerp(
			lerp(b00, b10, x - loX),
			lerp(b01, b11, x - loX),
			y - loY);

		return lerp(a, b, z - loZ);
	}

	sampleNearest(u, v, w) {
		const x = Math.round(u * this.width);
		const y = Math.round(v * this.height);
		const z = Math.round(w * this.depth);

		return this.get(x, y, z);
	}

	get(x, y, z) {
		return this.a[this.wrap(x, y, z)];
	}

	set(x, y, z, v) {
		this.a[this.index(x, y, z)] = v;
	}

	setNearest(u, v, w, value) {
		const x = Math.round(u * this.width);
		const y = Math.round(v * this.height);
		const z = Math.round(w * this.depth);
		this.set(x, y, z, value);
		console.log([x, y, z]);
	}

	/** Gets the index of the given coordinates. */
	index(x, y, z) {
		return (
			x +
			y * this.width +
			z * this.width * this.height);
	}

	/** Gets the index of the given coordinate wraping around the edges. */
	wrap(x, y, z) {
		return (
			mod(x, this.width) +
			mod(y, this.height) * this.width + 
			mod(z, this.depth) * this.width * this.height);
	}
}

class DoubleBuffer extends Buffer {
	constructor(width, height, depth) {
		super(width, height, depth)
		this.b = new Float32Array(this.a.length);
	}

	swap() {
		const t = this.a;
		this.a = this.b;
		this.b = t;
	}

	out(x, y, z, v) {
		return this.b[this.index(x, y, z)] = v;
	}

	outNearest(u, v, value) {
		const x = Math.round(u * this.width);
		const y = Math.round(v * this.height);
		const z = Math.round(z * this.depth);
		this.out(x, y, z, value);
	}
}

class Simulation {
	constructor(width, height, depth) {
		this.width = width;
		this.height = height;
		this.depth = depth

		this.velocityX = new DoubleBuffer(width, height, depth);
		this.velocityY = new DoubleBuffer(width, height, depth);
		this.velocityZ = new DoubleBuffer(width, height, depth);
		this.curl = new Buffer(width, height, depth);
		this.divergence = new Buffer(width, height, depth);
		this.pressure = new DoubleBuffer(width, height, depth);

		this.iterations = 4;

		this.curlFactor = 0.1;
	}

	step(dt) {
		this.dt = dt;
		//console.log("+");

		//this.stepCurl();
		//this.stepVorticity();
		this.stepDivergence();
		this.stepPressure();
		this.stepSubtractGradient();
		this.stepAdvection();

	}

	stepCurl() {
		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				for (let k = 0; k < this.depth; k++) {
					const w = -this.velocityY.get(i - 1, j, k);
					const e = +this.velocityY.get(i + 1, j, k);
					const n = +this.velocityX.get(i, j - 1, k);
					const s = -this.velocityX.get(i, j + 1, k);
					const u = +this.velocityX.get(i, j, k - 1);
					const d = -this.velocityX.get(i, j, k + 1);

					const curl = 0.5 * (e + w + n + s + u + d);
					this.curl.set(i, j, k, curl);
				}
			}
		}
	}

	stepVorticity() {
		const curlFactor = config.vorticity;

		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				for (let k = 0; k < this.depth; k++) {
					const w = this.curl.get(i - 1, j, k);
					const e = this.curl.get(i + 1, j, k);
					const n = this.curl.get(i, j - 1, k);
					const s = this.curl.get(i, j + 1, k);
					const u = this.curl.get(i, j, k - 1);
					const d = this.curl.get(i, j, k + 1);

					let forceX = Math.abs(s) - Math.abs(n);
					let forceY = Math.abs(e) - Math.abs(w);
					let forceZ = Math.abs(d) - Math.abs(u);
					
					// normalize force
					const length = Math.sqrt(forceX * forceX + forceY * forceY) || 0.0001;

					if (isNaN(forceX)) { 
						console.log([forceX, forceY]);
						debugger;
					}

					forceX = forceX / length;
					forceY = forceY / length;


					const c = this.curl.get(i, j);
					forceX *= -curlFactor * c;
					forceY *= curlFactor * c;

					const velocityX = this.velocityX.get(i, j);
					const velocityY = this.velocityY.get(i, j);

					this.velocityX.out(i, j, k, velocityX + forceX * this.dt);
					this.velocityY.out(i, j, k, velocityY + forceY * this.dt);
				}
			}
		}

		this.velocityX.swap();
		this.velocityY.swap();
		this.velocityZ.swap();
	}

	/**
	 * Calculates the amount of divergence
	 *  + = divergence
	 *  - = convergence
	 */
	stepDivergence() {
		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				for (let k = 0; k < this.depth; k++) {
					const w = -this.velocityX.get(i - 1, j, k);
					const e = +this.velocityX.get(i + 1, j, k);
					const n = -this.velocityY.get(i, j - 1, k);
					const s = +this.velocityY.get(i, j + 1, k);
					const u = -this.velocityZ.get(i, j, k - 1);
					const d = +this.velocityZ.get(i, j, k + 1);

					const divergence = 0.5 * (e + w + n + s + u + d);
					this.divergence.set(i, j, k, divergence);
				}
			}
		}
	}

	stepPressure() {
		this.pressure.clear(config.pressure);

		for (let iter = 0; iter < this.iterations; iter++) {
			
			for (let i = 0; i < this.width; i++) {
				for (let j = 0; j < this.height; j++) {
					for (let k = 0; k < this.depth; k++) {
						const w = this.pressure.get(i - 1, j, k);
						const e = this.pressure.get(i + 1, j, k);
						const n = this.pressure.get(i, j - 1, k);
						const s = this.pressure.get(i, j + 1, k);
						const u = this.pressure.get(i, j, k - 1);
						const d = this.pressure.get(i, j, k + 1);
						//const c = this.pressure.get(i, j, k);
						const divergence = this.divergence.get(i, j, k);
						const pressure = (e + w + n + s + u + d - divergence) / 8;
						this.pressure.out(i, j, k, pressure);
					}
				}
			}

			this.pressure.swap();
		}

		// Swap back after the last iteration.
		this.pressure.swap();
	}

	stepSubtractGradient() {
		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				for (let k = 0; k < this.depth; k++) {	
					const w = this.pressure.get(i - 1, j, k);
					const e = this.pressure.get(i + 1, j, k);
					const n = this.pressure.get(i, j - 1, k);
					const s = this.pressure.get(i, j + 1, k);
					const u = this.pressure.get(i, j, k - 1);
					const d = this.pressure.get(i, j, k + 1);

					const x0 = this.velocityX.get(i, j, k);
					const y0 = this.velocityY.get(i, j, k);
					const z0 = this.velocityZ.get(i, j, k);

					const x = x0 - e + w;
					const y = y0 - s + n;
					const z = z0 - d + u;

					this.velocityX.out(i, j, k, x);
					this.velocityY.out(i, j, k, y);
					this.velocityZ.out(i, j, k, z);
				}
			}
		}

		this.velocityX.swap();
		this.velocityY.swap();
		this.velocityZ.swap();
	}

	stepAdvection() {
		const decay = 0.99;

		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				for (let k = 0; k < this.depth; k++) {	
					const velocityX = this.velocityX.get(i, j, k);
					const velocityY = this.velocityY.get(i, j, k);
					const velocityZ = this.velocityZ.get(i, j, k);

					const u = (i - velocityX * this.dt) / this.width;
					const v = (j - velocityY * this.dt) / this.height;
					const w = (k - velocityZ * this.dt) / this.depth;

					this.velocityX.out(i, j, k, this.velocityX.sample(u, v, w) * decay);
					this.velocityY.out(i, j, k, this.velocityY.sample(u, v, w) * decay);
					this.velocityZ.out(i, j, k, this.velocityZ.sample(u, v, w) * decay);
				}
			}
		}

		this.velocityX.swap();
		this.velocityY.swap();
		this.velocityZ.swap();
	}
}

class Visualizer {
	constructor(width, height, id) {
		this.channels = 4;

		this.width = width;
		this.height = height;

		this.canvas = document.getElementById(id);

		this.dpr = (window.devicePixelRatio || 1) / 8;
		this.canvas.width = width;
		this.canvas.height = height;
		this.canvas.style.width = `${width / this.dpr}px`;
		this.canvas.style.height = `${height / this.dpr}px`;

		this.context = canvas.getContext('2d');	
		this.context.fillStyle = 'red';
		this.context.fillRect(0, 0, width, height);

		this.source = this.context.getImageData(0, 0, width, height);
		this.destination = this.context.getImageData(0, 0, width, height);

		this.swap();

		this.sample = this.sampleBilinear;
	}

	swap() {
		const swap = this.source;
		this.source = this.destination;
		this.destination = swap;
	}

	step(dt, simulation) {
		this.dt = dt;

		switch(config.display) {
			case 'velocity':
				this.stepOut(simulation.velocityX, simulation.velocityY, simulation.velocityZ);
				break;
			case 'pressure':
				this.stepOut(simulation.pressure);
				break;
			case 'curl':
				this.stepOut(simulation.curl);
				break;
			case 'divergence':
				this.stepOut(simulation.divergence);
				break;
		}

		this.context.putImageData(this.source, 0, 0);

		this.swap();
	}

	get(x, y, c) {
		return this.source.data[this.wrap(x, y) * 4 + c];
	}

	set(x, y, c, v) {
		this.destination.data[this.index(x, y) * 4 + c] = v;
	}

	/** Gets the index of the given coordinates. */
	index(x, y) {
		return x + y * this.width;
	}

	/** Gets the index of the given coordinate wraping around the edges. */
	wrap(x, y) {
		return mod(x, this.width) + mod(y, this.height) * this.width;
	}

	setColor(x, y, r, g, b) {
		const o= this.index(x, y) * 4;
		this.destination.data[o + 0] = r;
		this.destination.data[o + 1] = g;
		this.destination.data[o + 2] = b;
	}

	stepOut(bufferR, bufferG, bufferB) {
		const w = this.width;
		const h = this.height;

		const scale = Math.pow(2, config.scale);
		const c = config.sliceDepth / side;

		switch (config.slice) {
			case 'xy':
				for (let i = 0; i < this.width; i++) {
					for (let j = 0; j < this.height; j++) {
						const u = i / w;
						const v = j / h;

						let r = (bufferR && bufferR.sample(u, v, c)) || 0;
						let g = (bufferG && bufferG.sample(u, v, c)) || 0;
						let b = (bufferB && bufferB.sample(u, v, c)) || 0;

						this.setColor(i, j, r * scale + 127, g * scale + 127, b * scale + 127);
					}
				}
				break;
			case 'xz':
				for (let i = 0; i < this.width; i++) {
					for (let j = 0; j < this.height; j++) {
						const u = i / w;
						const v = j / h;

						let r = (bufferR && bufferR.sample(u, c, v)) || 0;
						let g = (bufferG && bufferG.sample(u, c, v)) || 0;
						let b = (bufferB && bufferB.sample(u, c, v)) || 0;

						this.setColor(i, j, r * scale + 127, g * scale + 127, b * scale + 127);
					}
				}
				break;
			case 'yz':
				for (let i = 0; i < this.width; i++) {
					for (let j = 0; j < this.height; j++) {
						const u = i / w;
						const v = j / h;

						let r = (bufferR && bufferR.sample(c, u, v)) || 0;
						let g = (bufferG && bufferG.sample(c, u, v)) || 0;
						let b = (bufferB && bufferB.sample(c, u, v)) || 0;

						this.setColor(i, j, r * scale + 127, g * scale + 127, b * scale + 127);
					}
				}
				break;
		}
	}
}

let simulation;
let visualizer;
let interactions = [];
const config = {
	display: 'velocity',
	scale: 10,
	pressure: 0,
	vorticity: 0,
	slice: 'xy',
	sliceDepth: 0,
}

const side = 64;
let frame = 0;

const gui = () => {
    const gui = new dat.GUI({ width: 300 });
    gui.remember(config);
    gui.add(config, 'display', ['velocity', 'pressure', 'curl', 'divergence']).name('Display');
    gui.add(config, 'scale', -10, 100).name('Display Scale');
    gui.add(config, 'slice', ['xy', 'xz', 'yz']).name('Slice');
    gui.add(config, 'sliceDepth', 0, side - 1).name('Slice Depth');
    gui.add(config, 'pressure', 0, 1).step(0.1);
    gui.add(config, 'vorticity', 0, 50);
}

const initialize = () => {
	const visualScale = 4;

	simulation = new Simulation(side, side, side);
	visualizer = new Visualizer(side, side, 'canvas');

	visualizer.canvas.addEventListener('mousemove', onMouseMove);

	simulation.velocityY.random();
	simulation.velocityX.random();
	simulation.velocityZ.random();

	gui();

	window.requestAnimationFrame(advance);
}

const onMouseMove = (e) => {
	const u = e.offsetX * visualizer.dpr / visualizer.width;
	const v = e.offsetY * visualizer.dpr / visualizer.height;

	const du = e.movementX * visualizer.dpr / visualizer.width;
	const dv = e.movementY * visualizer.dpr / visualizer.height;


	interactions.push({u, v, du, dv});
}

const advance = () => {
	console.log(`Frame ${frame++}`)
	if (interactions.length > 0) {
		const {u, v, du, dv} = interactions[0];
		const c = config.sliceDepth / side;

		const len = Math.sqrt(du * du + dv * dv) || 1;

		const vx = du / len * 40;
		const vy = dv / len * 40;

		switch (config.slice) {
			case 'xy':
				for (let i = -3; i <= 3; i++) {
					for (let j = -3; j <= 3; j++) {
						const uu = i / simulation.width;
						const vv = j / simulation.height;

						simulation.velocityX.setNearest(u + uu, v + vv, c, vx);
						simulation.velocityY.setNearest(u + uu, v + vv, c, vy);
					}
				}	
				break;
			case 'xz':
				for (let i = -3; i <= 3; i++) {
					for (let j = -3; j <= 3; j++) {
						const uu = i / simulation.width;
						const vv = j / simulation.depth;

						simulation.velocityX.setNearest(u + uu, c, v + vv, vx);
						simulation.velocityZ.setNearest(u + uu, c, v + vv, vy);
					}
				}	
				break;
			case 'yz':
				for (let i = -3; i <= 3; i++) {
					for (let j = -3; j <= 3; j++) {
						const uu = i / simulation.height;
						const vv = j / simulation.depth;

						simulation.velocityY.setNearest(c, u + uu, v + vv, vx);
						simulation.velocityZ.setNearest(c, u + uu, v + vv, vy);
					}
				}	
				break;
		}

		console.log([u, v, c, vx, vy])


		interactions = [];
	}

	simulation.step(0.1);
	visualizer.step(0.1, simulation);
	window.requestAnimationFrame(advance);
};

window.onload = initialize;