package snail;
import snail.justGraphix.IPathContext;

// --- MULTI-TARGET BUFFER ALIAS ---
#if (cpp || cppia || hl || jvm || java)
    typedef Buffer = haxe.ds.Vector<Float>;
#else
    typedef Buffer = Array<Float>;
#end

enum abstract OptimizationMode(Int) {
    var Performance;
    var Accurate;
    var Adaptive;
}

class QuadCurveBuilder implements IPathContext {
    public var curves( default, null ): Buffer;
    public var curveBounds( default, null ): Buffer;
    public var indices(default, null):Array<Int>;
    public var pointers(default, null):Array<Int>;
    var count:Int = 0;
    // pen position
    var ax: Float = 0.;
    var ay: Float = 0.;
    public function new( size: Int = 2048, bands: Int = 32 ) {
        #if (cpp || cppia || hl || jvm || java)
            curves = new haxe.ds.Vector<Float>( size );
            curveBounds = new haxe.ds.Vector<Float>( Std.int(size / 2) );
        #else
            curves = new Array<Float>();
            curveBounds = new Array<Float>();
        #end
        indices = new Array<Int>();
        pointers = new Array<Int>();
        pointers.resize(bands * 2);
    }
    public inline
    function clear():Void {
        count = 0;
        #if !(cpp || cppia || hl || jvm || java)
            this.curves.resize( 0 );
            this.curveBounds.resize( 0 );
        #end
    }
    public var length(get, null): Int;
    private inline
    function get_length():Int { 
        return count << 2; 
    }
    
    public inline
    function pushYbounds( ay: Float, by: Float, cy: Float ): Void {
        var i = count << 1; 
        var ab = (ay < by); 
        var ac = (ay < cy); 
        var bc = (by < cy);
        curveBounds[i]     = ab ? (ac ? ay : cy) : (bc ? by : cy);
        curveBounds[i + 1] = ab ? (bc ? cy : by) : (ac ? cy : ay);
    }
    public function calcBands( bands: Int, minY: Float, maxY: Float ): Void {
        var n = count;
        if (n == 0) return;
        var b = curveBounds;
        var h = maxY - minY;
        if (h <= 0.0) h = 1.0;
        var stride = h / bands;
        var buckets = [for (i in 0...bands) new Array<Int>()];
        // Scan and bin curve IDs into row buckets
        for( i in 0...n ){
            var j = i << 1;
            var y0 = b[ j ];
            var y1 = b[ j + 1] ;
            var r0 = Std.int(( y0 - minY ) / stride);
            var r1 = Std.int(( y1 - minY ) / stride);
            if( r0 < 0 ) r0 = 0;
            if( r1 >= bands ) r1 = bands - 1;
            for( r in r0...( r1 + 1 ) ) buckets[ r ].push( i );
        }
        this.indices.resize(0);
        if (this.pointers.length != bands * 2) {
            this.pointers.resize(bands * 2);
        }
        // Flatten directly into the class properties
        var offset = 0;
        for( r in 0...bands ){
            var bucket = buckets[ r ];
            var len = bucket.length;
            var k = r << 1;
            pointers[ k ]     = offset;
            pointers[ k + 1 ] = len;
            for( i in 0...len ) indices.push( bucket[ i ] );
            offset += len;
        }
    }
    public inline
    function moveTo( x: Float, y: Float ){
        ax = x;
        ay = y;
    }
    public inline
    function lineTo( bx: Float, by: Float ): Int {
        var idx = count << 2;
        // midpoint
        var qx = (ax + bx) * 0.5;
        var qy = (ay + by) * 0.5;
        curves[ idx ] = qx;
        curves[ idx + 1 ] = qy;
        curves[ idx + 2 ] = bx;
        curves[ idx + 3 ] = by;
        pushYbounds( ay, qy, by );
        // update pen
        ax = bx;
        ay = by;
        count++;
        return 4;
    }
    public inline
    function quadTo( bx: Float, by: Float
                   , cx: Float, cy: Float ): Int {
        var idx = count << 2;
        curves[ idx ] = bx;
        curves[ idx + 1 ] = by;
        curves[ idx + 2 ] = cx;
        curves[ idx + 3 ] = cy;
        pushYbounds( ay, by, cy );
        // update pen
        ax = cx;
        ay = cy;
        count++;
        return 4;
    }
    public inline 
    function cubicTo(  bx: Float, by: Float
                     , cx: Float, cy: Float
                     , dx: Float, dy: Float ): Int {
        return addCubicCurve( bx, by, cx, cy, dx, dy, Adaptive );
    }
    public inline function addCubicCurve(  bx: Float, by: Float
                                         , cx: Float, cy: Float
                                         , dx: Float, dy: Float
                                         , mode: OptimizationMode
                                         , errorMargin: Float = 0.5
                                         , maxDepth: Int = 5 ): Int {
        var startIndex = count;
        var runAccurate = ( mode == Accurate );
        if( mode == Adaptive ) {
            var dx1 = bx - ax;
            var dy1 = by - ay; 
            var dx2 = dx - cx;
            var dy2 = dy - cy; 
            var crossHandles = dx1 * dy2 - dy1 * dx2;
            var dxBase = dx - ax;
            var dyBase = dy - ay;
            var side1 = dxBase * (by - ay) - dyBase * (bx - ax);
            var side2 = dxBase * (cy - ay) - dyBase * (cx - ax);
            var isProblematic = ( side1 * side2 < -0.01 ) || (Math.abs( crossHandles ) < 0.05 && ( dx1 * dx1 + dy1 * dy1 > 1.0 ));
            if( isProblematic ){
                runAccurate = true;
                maxDepth += 1;
            }
        }
        if( runAccurate || mode == Accurate ){
            var errorMarginSquared = errorMargin * errorMargin;
            subdivideAccurate( this, ax, ay, bx, by, cx, cy, dx, dy, errorMarginSquared, 0, maxDepth );
        } else {
            subdividePerformance( this, ax, ay, bx, by, cx, cy, dx, dy, 0, 3 );
        }
        ax = dx;
        ay = dy;
        return count - startIndex; 
    }

    static function subdivideAccurate(  builder: QuadCurveBuilder, px: Float, py: Float
                               , bx: Float, by: Float
                               , cx: Float, cy: Float
                               , dx: Float, dy: Float
                               , errorMarginSquared: Float
                               , currentDepth: Int
                               , maxDepth: Int
    ):Void {
        var curves = builder.curves;
        var dx_ = dx - 3.0 * cx + 3.0 * bx - px;
        var dy_ = dy - 3.0 * cy + 3.0 * by - py;
        var estimatedErrorSquared = (dx_ * dx_ + dy_ * dy_) * 0.009259259259259259;
        if( estimatedErrorSquared <= errorMarginSquared || currentDepth >= maxDepth ){
            var qx = (3.0 * bx - px + 3.0 * cx - dx) * 0.25;
            var qy = (3.0 * by - py + 3.0 * cy - dy) * 0.25;
            var idx = builder.count << 2;
            curves[ idx ] = qx;
            curves[ idx + 1 ] = qy;
            curves[ idx + 2 ] = dx;
            curves[ idx + 3 ] = dy;
            builder.pushYbounds( py, qy, dy );
            builder.count++;
            return;
        }
        var midL1x = ( px + bx ) * 0.5; 
        var midL1y = ( py + by ) * 0.5;
        var midMx  = ( bx + cx ) * 0.5;
        var midMy  = ( by + cy ) * 0.5;
        var midR2x = ( cx + dx ) * 0.5;
        var midR2y = ( cy + dy ) * 0.5;
        var midL2x = ( midL1x + midMx ) * 0.5;
        var midL2y = ( midL1y + midMy ) * 0.5;
        var midR1x = ( midMx + midR2x ) * 0.5;
        var midR1y = ( midMy + midR2y ) * 0.5;
        var splitX = ( midL2x + midR1x ) * 0.5;
        var splitY = ( midL2y + midR1y ) * 0.5;
        var nextDepth = currentDepth + 1;
        subdivideAccurate( builder, px, py, midL1x, midL1y, midL2x, midL2y, splitX, splitY, errorMarginSquared, nextDepth, maxDepth );
        subdivideAccurate( builder, splitX, splitY, midR1x, midR1y, midR2x, midR2y, dx, dy, errorMarginSquared, nextDepth, maxDepth );
    }

    static function subdividePerformance(  builder: QuadCurveBuilder, px: Float, py: Float 
                                  , bx: Float, by: Float
                                  , cx: Float, cy: Float
                                  , dx: Float, dy:Float
                                  , currentDepth:Int, maxDepth:Int ):Void {
        var curves = builder.curves;
        if( currentDepth >= maxDepth ){
            var qx = ( 3.0 * bx - px + 3.0 * cx - dx ) * 0.25;
            var qy = ( 3.0 * by - py + 3.0 * cy - dy ) * 0.25;
            var idx = builder.count << 2;
            curves[ idx ] = qx;
            curves[ idx + 1 ] = qy;
            curves[ idx + 2 ] = dx;
            curves[ idx + 3 ] = dy;
            builder.pushYbounds( py, qy, dy );
            builder.count++;
            return;
        }
        var midL1x = ( px + bx ) * 0.5;
        var midL1y = ( py + by ) * 0.5;
        var midMx  = ( bx + cx ) * 0.5;
        var midMy  = ( by + cy ) * 0.5;
        var midR2x = ( cx + dx ) * 0.5;
        var midR2y = ( cy + dy ) * 0.5;
        var midL2x = ( midL1x + midMx ) * 0.5;
        var midL2y = ( midL1y + midMy ) * 0.5;
        var midR1x = ( midMx + midR2x ) * 0.5;
        var midR1y = ( midMy + midR2y ) * 0.5;
        var splitX = ( midL2x + midR1x ) * 0.5;
        var splitY = ( midL2y + midR1y ) * 0.5;
        var nextDepth = currentDepth + 1;
        subdividePerformance( builder, px, py, midL1x, midL1y, midL2x, midL2y, splitX, splitY, nextDepth, maxDepth );
        subdividePerformance( builder, splitX, splitY, midR1x, midR1y, midR2x, midR2y, dx, dy, nextDepth, maxDepth );
    }
}
