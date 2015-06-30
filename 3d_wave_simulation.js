//--- 2D simulation code
var plane;

var WaveSpeed = 0.5;

var WorldSize = 10.0;
var NX = 200;
var NY = NX;
var ArraySize = NX * NX; //65536
var DXY = WorldSize / NX;


var LX = WorldSize;
var LY = WorldSize;
var LZ = WorldSize / 2.0;


var StateSize = 8;
var State = [];


var StateHeight = [];
var StateVel = [];
var StateHeightPrev = [];
var StateVelPrev = [];
var StateVelStar = [];
var StateAccelStar = [];
var StateHeightStar = [];
var StateJacobiTmp = [];

var StateHeightIndex = 0;
var StateVelIndex = 1;
var StateHeightPrevIndex = 2;
var StateVelPrevIndex = 3;
var StateVelStarIndex = 4;
var StateAccelStarIndex = 5;
var StateHeightStarIndex = 6;
var StateJacobiTmpIndex = 7;

State[0] = StateHeight;
State[1] = StateVel;
State[2] = StateHeightPrev;
State[3] = StateVelPrev;
State[4] = StateVelStar;
State[5] = StateAccelStar;
State[6] = StateHeightStar;
State[7] = StateJacobiTmp;

var StateCurrentTime = 0.0;

var InputActive = false;
var InputIndexX = 0;
var InputIndexY = 0;
var InputHeight = 0;

var mouse = new THREE.Vector2()

var particle_count = ArraySize;
var halfParticle_count = NX/2.0;

//---





var SEPARATION = 20, AMOUNTX = NX, AMOUNTY = NY;

var renderer, scene, camera, stats;

var particleSystem, uniforms, geometry;


var WIDTH = window.innerWidth;
var HEIGHT = window.innerHeight;

init();
animate();

function init() {

	camera = new THREE.PerspectiveCamera( 40, WIDTH / HEIGHT, 1, 10000 );
	camera.position.z = 500;

	scene = new THREE.Scene();

	var attributes = {

		size:        { type: 'f', value: null },
		customColor: { type: 'c', value: null }

	};

	uniforms = {

		color:     { type: "c", value: new THREE.Color( 0xffffff ) },
		texture:   { type: "t", value: THREE.ImageUtils.loadTexture( "img/spark1.png" ) }

	};

	var shaderMaterial = new THREE.ShaderMaterial( {

		uniforms:       uniforms,
		attributes:     attributes,
		vertexShader:   document.getElementById( 'vertexshader' ).textContent,
		fragmentShader: document.getElementById( 'fragmentshader' ).textContent,

		blending:       THREE.AdditiveBlending,
		depthTest:      false,
		transparent:    true

	});


	var radius = 200;

	geometry = new THREE.BufferGeometry();

	
	var positions = new Float32Array( particle_count * 3 );
	var values_color = new Float32Array( particle_count * 3 );
	var values_size = new Float32Array( particle_count );

	var color = new THREE.Color();

	//for( var v = 0; v < particle_count; v++ )
	//{
		var v = 0;
		//debugger;
		//console.log(halfParticles);
		for ( var ix=-NX/2; ix<NX/2; ix++)
		{
			//var iy = 0;
			for ( var iy=-NX/2; iy<NX/2; iy++)
			{
				var iz = 0;
				//for ( var iz=-40; iz<40; iz++)
				{
					values_size[ v ] = 1;
					//console.log("(", ix*5, ",", iy*5, ")");
					//positions[ v * 3 + 0 ] = ( Math.random() * 2 - 1 ) * radius;
					//positions[ v * 3 + 1 ] = ( Math.random() * 2 - 1 ) * radius;
					//positions[ v * 3 + 2 ] = ( Math.random() * 2 - 1 ) * radius;

					positions[ v * 3 + 0 ] = ix*5;
					positions[ v * 3 + 1 ] = iy*5;
					positions[ v * 3 + 2 ] = iz*5;



					color.setHSL( v / particle_count, 1.0, 0.5 );

					values_color[ v * 3 + 0 ] = color.r;
					values_color[ v * 3 + 1 ] = color.g;
					values_color[ v * 3 + 2 ] = color.b;

					v++;

					if (!(v<particle_count))
					{
						break;
					}
				}
			}	
		}


	geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
	geometry.addAttribute( 'customColor', new THREE.BufferAttribute( values_color, 3 ) );
	geometry.addAttribute( 'size', new THREE.BufferAttribute( values_size, 1 ) );

	particleSystem = new THREE.PointCloud( geometry, shaderMaterial );
	particleSystem.rotation.x = -70 * Math.PI / 180.0
	scene.add( particleSystem );

	renderer = new THREE.WebGLRenderer();
	renderer.setPixelRatio( window.devicePixelRatio );
	renderer.setSize( WIDTH, HEIGHT );

	var container = document.getElementById( 'container' );
	container.appendChild( renderer.domElement );

	stats = new Stats();
	stats.domElement.style.position = 'absolute';
	stats.domElement.style.top = '0px';
	container.appendChild( stats.domElement );

	setup()


	//
	document.addEventListener( 'mousedown', onDocumentMouseDown, false );

	window.addEventListener( 'resize', onWindowResize, false );

}


function onDocumentMouseDown( event ) {

	//event.preventDefault();
	console.log('Press');
	var iX = NX/2;
    var iY = NX/2;
    if ( iX > 0 && iX < NX-1 && iY > 0 && iY < NY-1 ) {
        InputIndexX = iX;
        InputIndexY = iY;
        InputHeight = 2500.0;
        InputActive = true;
    }

    return;
}
		// Index an element of a grid in the state array
function IX( i, j )
{
	//debugger;
	var p = i + NX*j;
	//console.log( p );
    return ( p );
}


function EnforceDirichletBoundaryConditions( io_a ) {
    for (var j = 0; j < NY; ++j) {
        if (j == 0 || j == (NY-1)) {
            for (var i = 0; i < NX; ++i) {
                State[io_a][IX(i,j)] = 0.0;
            }
        } else {
            State[io_a][IX(0,j)] = 0.0;
            State[io_a][IX(NX-1,j)] = 0.0;
        }
    }
}

function EnforceNeumannBoundaryConditions( io_v ) {
    for (var j = 0; j < NY; ++j) {
        if (j == 0) {
            for (var i = 0; i < NX; ++i) {
                State[io_v][IX(i,0)] = State[io_v][IX(i,1)];
            }
        } else if (j == (NY-1)) {
            for (var i = 0; i < NX; ++i) {
                State[io_v][IX(i,NY-1)] = State[io_v][IX(i,NY-2)];
            }
        }

        State[io_v][IX(0,j)] = State[io_v][IX(1,j)];
        State[io_v][IX(NX-1,j)] = State[io_v][IX(NX-2,j)];
    }
}

function EnforceHeightBoundaryConditions( io_h ) {
    EnforceNeumannBoundaryConditions(io_h);

    if ( InputActive ) {
        State[io_h][IX(InputIndexX, InputIndexY)] = InputHeight;
    }
    InputActive = false;
}



function CopyArray( i_src, o_dst )
{
    for ( var i = 0; i < ArraySize; i++ )
    {
        State[o_dst][i] = State[i_src][i];
    }
    
}

function FillArray( o_a, i_val )
{
    for ( var i = 0; i < ArraySize; i++ )
    {
        State[o_a][i] = i_val;
    }
}

function SetInitialState()
{
    for (var j = 0; j < NY; j++)
    {
	    for (var i = 0; i < NX; i++)
	    {
        	State[StateHeightIndex][IX(i,j)] = 0.5;
        	State[StateVelIndex][IX(i,j)] = 0.0;
    	}
    }
    EnforceHeightBoundaryConditions( StateHeightIndex );
    EnforceNeumannBoundaryConditions( StateVelIndex );
    StateCurrentTime = 0.0;

    CopyArray( StateHeightIndex, StateHeightPrevIndex );
    CopyArray( StateVelIndex, StateVelPrevIndex );
    InputActive = false;
    
}

function setup()
{
    SetInitialState();
}

function SwapHeight()
{
    var tmp = StateHeightIndex;
    StateHeightIndex = StateHeightPrevIndex;
    StateHeightPrevIndex = tmp;
}

function SwapVel()
{
    var tmp = StateVelIndex;
    StateVelIndex = StateVelPrevIndex;
    StateVelPrevIndex = tmp;
}

function SwapState()
{
    SwapHeight();
    SwapVel();
}


// Estimate height star
function EstimateHeightStar( i_dt )
{
    for ( var i = 0; i < ArraySize; i++ )
    {
        State[StateHeightStarIndex][i] = State[StateHeightPrevIndex][i] + 
                ( i_dt * State[StateVelStarIndex][i] );
    }
    EnforceHeightBoundaryConditions( StateHeightStarIndex );
}

// Estimate vel star
function EstimateVelStar( i_dt )
{
    for ( var i = 0; i < ArraySize; i++ )
    {
        State[StateVelStarIndex][i] = State[StateVelPrevIndex][i] + 
                ( i_dt * State[StateAccelStarIndex][i] );
    }
    EnforceNeumannBoundaryConditions( StateVelStarIndex );
}

// Jacobi iteration to get temp acceleration
function JacobiIterationAccel( i_aOld, o_aNew, i_hStar, i_dt )
{
    var kappa = Math.pow( WaveSpeed, 2 ) * Math.pow( i_dt, 2 ) / Math.pow( DXY, 2 );
    var gamma = Math.pow( WaveSpeed, 2 ) / Math.pow( DXY, 2 );

    for (var j = 1; j < NY-1; j++) {
        for (var i = 1; i < NX-1; i++) {
            var a_left = State[i_aOld][IX(i-1,j)];
            var a_right = State[i_aOld][IX(i+1,j)];
            var a_down = State[i_aOld][IX(i,j-1)];
            var a_up = State[i_aOld][IX(i,j+1)];

            var h_star_left = State[i_hStar][IX(i-1,j)];
            var h_star_right = State[i_hStar][IX(i+1,j)];
            var h_star_down = State[i_hStar][IX(i,j-1)];
            var h_star_up = State[i_hStar][IX(i,j+1)];
            var h_star_cen = State[i_hStar][IX(i,j)];

            var b = gamma *
                (h_star_left + h_star_right + h_star_down + h_star_up -
                 (4.0 * h_star_cen));

            var c = kappa * (a_left + a_right + a_down + a_up);

            State[o_aNew][IX(i,j)] = (b + c) / (1.0 + kappa);
        }
    }

    EnforceDirichletBoundaryConditions( o_aNew );
    
}

// Solve for acceleration.
function JacobiSolveAccel( i_hStar, i_dt )
{
    // Initialize acceleration to zero.
    FillArray( StateAccelStarIndex, 0.0 );
    
    // Solve from StateJacobiTmp into StateAccel
    for ( var iter = 0; iter < 20; iter++ )
    {
        var tmp = StateAccelStarIndex;
        StateAccelStarIndex = StateJacobiTmpIndex;
        StateJacobiTmpIndex = tmp;
        
        JacobiIterationAccel( StateJacobiTmpIndex, StateAccelStarIndex, i_hStar, i_dt );
    }
}

function EstimateAccelStar( i_dt )
{
    JacobiSolveAccel( StateHeightStarIndex, i_dt );
}

// Accumulate estimate
function AccumulateEstimate( i_dt )
{
    for ( var i = 0; i < ArraySize; i++ )
    {
        State[StateHeightIndex][i] += i_dt * State[StateVelStarIndex][i];
        State[StateVelIndex][i] += i_dt * State[StateAccelStarIndex][i];
    }
}

// Time Step function.
function TimeStep( i_dt )
{
	
    // Swap state
    SwapState();
    
    // Initialize estimate. This just amounts to copying
    // The previous values into the current values.
    CopyArray( StateHeightPrevIndex, StateHeightIndex );
    CopyArray( StateVelPrevIndex, StateVelIndex );
    
    // 1
    CopyArray( StateVelIndex, StateVelStarIndex );
    EstimateHeightStar( i_dt );
    EstimateAccelStar( i_dt );
    AccumulateEstimate( i_dt / 6.0 );
    
    // 2
    EstimateVelStar( i_dt / 2.0 );
    EstimateHeightStar( i_dt / 2.0 );
    EstimateAccelStar( i_dt / 2.0 );
    AccumulateEstimate( i_dt / 3.0 );
    
    // 3
    EstimateVelStar( i_dt / 2.0 );
    EstimateHeightStar( i_dt / 2.0 );
    EstimateAccelStar( i_dt / 2.0 );
    AccumulateEstimate( i_dt / 3.0 );
    
    // 4
    EstimateVelStar( i_dt );
    EstimateHeightStar( i_dt );
    EstimateAccelStar( i_dt );
    AccumulateEstimate( i_dt / 6.0 );
    
    // Final boundary conditions on height and vel
    EnforceHeightBoundaryConditions( StateHeightIndex );
    EnforceNeumannBoundaryConditions( StateVelIndex );

    // Update current time.
    StateCurrentTime += i_dt;
    
}






function onWindowResize() {

	windowHalfX = window.innerWidth / 2;
	windowHalfY = window.innerHeight / 2;

	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();

	renderer.setSize( window.innerWidth, window.innerHeight );

}

//

//

function animate() {

	requestAnimationFrame( animate );

	render();
	stats.update();

}

function render() {

		TimeStep( 1.0 / 24.0 );


	var i = 0;

	for ( var ix = 0; ix < AMOUNTX; ix ++ ) {

		for ( var iy = 0; iy < AMOUNTY; iy ++ ) {

			particle = particles[ i++ ];
			particle.position.y = 0.0


			particle.scale.x = 20+State[StateHeightIndex][IX(ix,iy)] * 10;
			particle.scale.y = 20+State[StateHeightIndex][IX(ix,iy)] * 10;
			particle.scale.z = 20+State[StateHeightIndex][IX(ix,iy)] * 10;            

		}

	}

	renderer.render( scene, camera );

	//count += 0.1;

}


function onWindowResize() {

	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();

	renderer.setSize( window.innerWidth, window.innerHeight );

}

function animate() {

	requestAnimationFrame( animate );

	render();
	stats.update();

}

function render() {

	var time = Date.now() * 0.005;
	
	TimeStep( 1.0 / 24.0 );

	var size = geometry.attributes.size.array;

	var i = 0;

	for ( var ix = 0; ix < AMOUNTX; ix ++ ) {

		for ( var iy = 0; iy < AMOUNTY; iy ++ ) {

			size[ i ] = 20+State[StateHeightIndex][IX(ix,iy)] * 10;
			i++
		}

	}
	geometry.attributes.size.needsUpdate = true;

	renderer.render( scene, camera );

/*
	//particleSystem.rotation.z = 0.01 * time;

	var size = geometry.attributes.size.array;

	for( var i = 0; i < particle_count; i++ ) {

		size[ i ] = 10 * ( 1 + Math.sin( 0.1 * i + time ) );

	}

	geometry.attributes.size.needsUpdate = true;

	renderer.render( scene, camera );
*/
}