

// http://jamie-wong.com/2016/08/05/webgl-fluid-simulation/
// https://www.esimov.com/2014/01/navier-stokes-fluid-simulation-on-html5-canvas

const mod = (n, d) => ((n % d) + d) % d;
const lerp = (a, b, s) => (a * (1 - s)) + (b * s);
const clamp = (v, lo, hi) => Math.max(Math.min(v, hi), lo);

class Buffer {
	constructor(width, height) {
		const size = width * height;
		this.width = width;
		this.height = height;
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
	sample(u, v) {
		const x = u * this.width;
		const y = v * this.height;

		const loX = Math.floor(x);
		const hiX = loX + 1;
		const loY = Math.floor(y);
		const hiY = loY + 1;

		const v00 = this.get(loX, loY);
		const v01 = this.get(loX, hiY);
		const v10 = this.get(hiX, loY);
		const v11 = this.get(hiX, hiY);

		return lerp(
			lerp(v00, v10, x - loX),
			lerp(v01, v11, x - loX),
			y - loY);
	}

	sampleNearest(u, v) {
		const x = Math.round(u * this.width);
		const y = Math.round(v * this.height);

		return this.get(x, y);
	}

	get(x, y) {
		return this.a[this.wrap(x, y)];
	}

	set(x, y, v) {
		this.a[this.index(x, y)] = v;
	}

	setNearest(u, v, value) {
		const x = Math.round(u * this.width);
		const y = Math.round(v * this.height);
		this.set(x, y, value);
	}

	/** Gets the index of the given coordinates. */
	index(x, y) {
		return x + y * this.width;
	}

	/** Gets the index of the given coordinate wraping around the edges. */
	wrap(x, y) {
		return mod(x, this.width) + mod(y, this.height) * this.width;
	}
}

class DoubleBuffer extends Buffer {
	constructor(width, height) {
		super(width, height)
		this.b = new Float32Array(this.a.length);
	}

	swap() {
		const t = this.a;
		this.a = this.b;
		this.b = t;
	}

	out(x, y, v) {
		return this.b[this.index(x, y)] = v;
	}

	outNearest(u, v, value) {
		const x = Math.round(u * this.width);
		const y = Math.round(v * this.height);
		this.out(x, y, value);
	}
}

class Simulation {
	constructor(width, height) {
		this.width = width;
		this.height = height;

		this.velocityX = new DoubleBuffer(width, height);
		this.velocityY = new DoubleBuffer(width, height);
		this.curl = new Buffer(width, height);
		this.divergence = new Buffer(width, height);
		this.pressure = new DoubleBuffer(width, height);

		this.iterations = 10;

		this.curlFactor = 0.1;
	}

	step(dt) {
		this.dt = dt;
		//console.log("+");

		this.stepCurl();
		this.stepVorticity();
		this.stepDivergence();
		this.stepPressure();
		this.stepSubtractGradient();
		this.stepAdvection();

	}

	stepCurl() {
		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				const w = -this.velocityY.get(i - 1, j);
				const e = +this.velocityY.get(i + 1, j);
				const n = +this.velocityX.get(i, j - 1);
				const s = -this.velocityX.get(i, j + 1);

				const curl = 0.5 * (e + w + n + s);
				this.curl.set(i, j, curl);
			}
		}
	}

	stepVorticity() {
		const curlFactor = config.vorticity;

		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				const w = this.curl.get(i - 1, j);
				const e = this.curl.get(i + 1, j);
				const n = this.curl.get(i, j - 1);
				const s = this.curl.get(i, j + 1);

				let forceX = Math.abs(s) - Math.abs(n);
				let forceY = Math.abs(e) - Math.abs(w);
				
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

				this.velocityX.out(i, j, velocityX + forceX * this.dt);
				this.velocityY.out(i, j, velocityY + forceY * this.dt);
			}
		}

		this.velocityX.swap();
		this.velocityY.swap();
	}

	/**
	 * Calculates the amount of divergence
	 *  + = divergence
	 *  - = convergence
	 */
	stepDivergence() {
		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				const w = -this.velocityX.get(i - 1, j);
				const e = +this.velocityX.get(i + 1, j);
				const n = -this.velocityY.get(i, j - 1);
				const s = +this.velocityY.get(i, j + 1);

				const divergence = 0.5 * (e + w + n + s);
				this.divergence.set(i, j, divergence);
			}
		}
	}

	stepPressure() {
		this.pressure.clear(config.pressure);

		for (let iter = 0; iter < this.iterations; iter++) {
			
			for (let i = 0; i < this.width; i++) {
				for (let j = 0; j < this.height; j++) {
					const w = this.pressure.get(i - 1, j);
					const e = this.pressure.get(i + 1, j);
					const n = this.pressure.get(i, j - 1);
					const s = this.pressure.get(i, j + 1);
					//const c = this.pressure.get(i, j);
					const divergence = this.divergence.get(i, j);
					const pressure = (e + w + n + s - divergence) / 4;
					this.pressure.out(i, j, pressure);
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
				const w = this.pressure.get(i - 1, j);
				const e = this.pressure.get(i + 1, j);
				const n = this.pressure.get(i, j - 1);
				const s = this.pressure.get(i, j + 1);

				const x0 = this.velocityX.get(i, j);
				const y0 = this.velocityY.get(i, j);

				const x = x0 - e + w;
				const y = y0 - s + n;

				this.velocityX.out(i, j, x);
				this.velocityY.out(i, j, y);
			}
		}

		this.velocityX.swap();
		this.velocityY.swap();
	}

	stepAdvection() {
		const decay = 0.99;

		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				const velocityX = this.velocityX.get(i, j);
				const velocityY = this.velocityY.get(i, j);

				const u = (i - velocityX * this.dt) / this.width;
				const v = (j - velocityY * this.dt) / this.height;

				this.velocityX.out(i, j, this.velocityX.sample(u, v) * decay);
				this.velocityY.out(i, j, this.velocityY.sample(u, v) * decay);
			}
		}

		this.velocityX.swap();
		this.velocityY.swap();
	}
}

class Visualizer {
	constructor(width, height, id) {
		this.channels = 4;

		this.width = width;
		this.height = height;

		this.canvas = document.getElementById(id);

		this.dpr = window.devicePixelRatio || 1;
		this.canvas.width = width;
		this.canvas.height = height;
		this.canvas.style.width = `${width / this.dpr}px`;
		this.canvas.style.height = `${height / this.dpr}px`;

		this.context = canvas.getContext('2d');	
		this.context.fillStyle = 'red';
		this.context.fillRect(0, 0, width, height);

		this.source = this.context.getImageData(0, 0, width, height);
		this.destination = this.context.getImageData(0, 0, width, height);

		this.checkerboard();
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
		this.vx = simulation.velocityX;
		this.vy = simulation.velocityY;

		switch(config.display) {
			case 'advection':
				this.stepAdvect();
				break;
			case 'velocity':
				this.stepOut(simulation.velocityX, simulation.velocityY);
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

	setColor(x, y, r, g, b) {
		const o= this.index(x, y) * 4;
		this.destination.data[o + 0] = r;
		this.destination.data[o + 1] = g;
		this.destination.data[o + 2] = b;
	}

	setNearest(u, v, c, value) {
		const x = Math.round(u * this.width);
		const y = Math.round(v * this.height);
		this.source.data[this.index(x, y) * 4 + c] = value;
		//this.set(x, y, c, value);
	}

 	/** Bilinear scaled sampling where u, v, [0, 1). */
	sampleBilinear(u, v) {
		const x = u * this.width;
		const y = v * this.height;

		const loX = Math.floor(x);
		const hiX = loX + 1;
		const loY = Math.floor(y);
		const hiY = loY + 1;

		const sX = x - loX;
		const sY = y - loY;

		const color = [0, 0, 0]

		for (let c = 0; c < this.channels - 1; c++) {
			const v00 = this.get(loX, loY, c);
			const v01 = this.get(loX, hiY, c);
			const v10 = this.get(hiX, loY, c);
			const v11 = this.get(hiX, hiY, c);

			color[c] = lerp(
				lerp(v00, v10, sX),
				lerp(v01, v11, sX),
				sY)
		}

		return color;
	}

	sampleNearest(u, v) {
		const x = Math.round(u * this.width);
		const y = Math.round(v * this.height);

		return [this.get(x, y, 0), this.get(x, y, 1), this.get(x, y, 2)]
	}

	/** Gets the index of the given coordinates. */
	index(x, y) {
		return x + y * this.width;
	}

	/** Gets the index of the given coordinate wraping around the edges. */
	wrap(x, y) {
		return mod(x, this.width) + mod(y, this.height) * this.width;
	}

	clear(r, g, b) {
		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				this.set(i, j, 0, r);
				this.set(i, j, 1, g);
				this.set(i, j, 2, b);
				this.set(i, j, 3, 255);
			}
		}
	}

	stepAdvect() {
		const w = this.width;
		const h = this.height;
		const s = this.dt * 40;

		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {

				const u = i / w;
				const v = j / h;

				//const velocityX = this.vx.sample(u, v);
				//const velocityY = this.vy.sample(u, v);
				const velocityX = this.vx.sample(u, v) / this.width;
				const velocityY = this.vy.sample(u, v) / this.height;

				const color = this.sample(u - s * velocityX, v - s * velocityY);
				//const color = this.sample(u, v);
				for (let c = 0; c < color.length; c++) {
					this.set(i, j, c, color[c]);
				}
			}
		}

	}

	stepOut(bufferR, bufferG, bufferB) {
		const w = this.width;
		const h = this.height;

		const scale = Math.pow(2, config.scale);

		for (let i = 0; i < this.width; i++) {
			for (let j = 0; j < this.height; j++) {
				const u = i / w;
				const v = j / h;

				let r = (bufferR && bufferR.sample(u, v)) || 0;
				let g = (bufferG && bufferG.sample(u, v)) || 0;
				let b = (bufferB && bufferB.sample(u, v)) || 0;

				//r = Math.sign(r) * Math.max(Math.log(Math.abs(r * scale)), 0);

				this.setColor(i, j, r * scale + 127, g * scale + 127, b * scale + 127);
			}
		}
	}



	checkerboard = () => {
		const width = 32;
		const height = 32;

		let column = false;
		for (let x = 0; x < this.width; x += width) {
			let row = column;
			column = !column;

			for (let y = 0; y < this.height; y += height) {
				const fill = (row) ? 255 : 0;
				for (let i = 0; i < width; i++) {
					for (let j = 0; j < height; j++) {
						this.set(x + i, y + j, 0, fill)
						this.set(x + i, y + j, 1, fill)
						this.set(x + i, y + j, 2, fill)
						this.set(x + i, y + j, 3, 255)
					}
				}

				row = !row;
			}
		}
	}
}

let simulation;
let visualizer;
let interactions = [];
const config = {
	display: 'advection',
	scale: 10,
	pressure: 0,
	vorticity: 0,
	checkerboard: () => {
		visualizer.checkerboard();
		visualizer.swap();
	}
}

const gui = () => {
    const gui = new dat.GUI({ width: 300 });
    gui.remember(config);
    gui.add(config, 'display', ['advection', 'velocity', 'pressure', 'curl', 'divergence']).name('Display');
    gui.add(config, 'scale', -10, 100).name('Display Scale');
    gui.add(config, 'pressure', 0, 1).step(0.1);
    gui.add(config, 'vorticity', 0, 50);
    gui.add(config, 'checkerboard').name('Checkerboard');
}

const initialize = () => {
	const side = 256;
	const visualScale = 4;

	simulation = new Simulation(side, side);
	visualizer = new Visualizer(side * visualScale, side * visualScale, 'canvas');

	visualizer.canvas.addEventListener('mousemove', onMouseMove);

	simulation.velocityY.random();
	simulation.velocityX.random();
	simulation.velocityY.clear(0);
	simulation.velocityX.clear(0);

	gui();

	window.requestAnimationFrame(advance);
}

const onMouseMove = (e) => {
	const u = e.offsetX * visualizer.dpr / visualizer.width;
	const v = e.offsetY * visualizer.dpr / visualizer.height;

	const du = e.movementX * visualizer.dpr / visualizer.width;
	const dv = e.movementY * visualizer.dpr / visualizer.height;

	console.log([e.movementX, e.movementY]);

	interactions.push({u, v, du, dv});
}

const advance = () => {

	if (interactions.length > 0) {
		const {u, v, du, dv} = interactions[0];

		const len = Math.sqrt(du * du + dv * dv) || 1;

		const vx = du / len * 40;
		const vy = dv / len * 40;

		for (let i = -3; i <= 3; i++) {
			for (let j = -3; j <= 3; j++) {
				const uu = i / simulation.width;
				const vv = j / simulation.height;

				simulation.velocityX.setNearest(u + uu, v + vv, vx);
				simulation.velocityY.setNearest(u + uu, v + vv, vy);
			}
		}

		console.log([vx, vy])


		interactions = [];
	}

	simulation.step(0.1);
	visualizer.step(0.1, simulation);
	window.requestAnimationFrame(advance);
};

window.onload = initialize;