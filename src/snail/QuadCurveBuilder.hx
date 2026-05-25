package snail;

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

class QuadCurveBuilder {
    public var curves( default, null ): Buffer;
    var count:Int = 0;
    // pen position
    var ax: Float = 0.;
    var ay: Float = 0.;
    public function new( size: Int = 2048 ) {
        #if (cpp || cppia || hl || jvm || java)
            this.curves = new haxe.ds.Vector<Float>( size );
        #else
            this.curves = new Array<Float>();
        #end
    }
    public inline
    function clear():Void {
        count = 0;
        #if !(cpp || cppia || hl || jvm || java)
            this.curves.resize( 0 );
        #end
    }
    public var length(get, null): Int;
    private inline
    function get_length():Int { 
        return count << 2; 
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
        // update pen
        ax = cx;
        ay = cy;
        count++;
        return 4;
    }
    public inline 
    function cubicTo(  bx: Float, by: Float
                     , cx: Float, dy: Float
                     , dx: Float, cy: Float ): Int {
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
            subdivideAccurate( bx, by, cx, cy, dx, dy, errorMarginSquared, 0, maxDepth );
        } else {
            subdividePerformance( bx, by, cx, cy, dx, dy, 0, 3 );
        }
        return count - startIndex; 
    }

    function subdivideAccurate(  bx: Float, by: Float
                               , cx: Float, cy: Float
                               , dx: Float, dy: Float
                               , errorMarginSquared: Float
                               , currentDepth: Int
                               , maxDepth: Int
    ):Void {
        var dx_ = dx - 3.0 * cx + 3.0 * bx - ax;
        var dy_ = dy - 3.0 * cy + 3.0 * by - ay;
        var estimatedErrorSquared = (dx_ * dx_ + dy_ * dy_) * 0.009259259259259259;
        if( estimatedErrorSquared <= errorMarginSquared || currentDepth >= maxDepth ){
            var qx = (3.0 * bx - ax + 3.0 * cx - dx) * 0.25;
            var qy = (3.0 * by - ay + 3.0 * cy - dy) * 0.25;
            var idx = count << 2;
            curves[ idx ] = qx;
            curves[ idx + 1 ] = qy;
            curves[ idx + 2 ] = dx;
            curves[ idx + 3 ] = dy;
            // update pen
            ax = dx;
            ay = dy;
            count++;
            return;
        }
        var midL1x = ( ax + bx ) * 0.5; 
        var midL1y = ( ay + by ) * 0.5;
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
        subdivideAccurate( midL1x, midL1y, midL2x, midL2y, splitX, splitY, errorMarginSquared, nextDepth, maxDepth );
        // make sure pen is correct
        ax = splitX;
        ay = splitY;
        subdivideAccurate( midR1x, midR1y, midR2x, midR2y, dx, dy, errorMarginSquared, nextDepth, maxDepth );
    }

    function subdividePerformance(  bx: Float, by: Float
                                  , cx: Float, cy: Float
                                  , dx: Float, dy:Float
                                  , currentDepth:Int, maxDepth:Int ):Void {
        if( currentDepth >= maxDepth ){
            var qx = ( 3.0 * bx - ax + 3.0 * cx - dx ) * 0.25;
            var qy = ( 3.0 * by - ay + 3.0 * cy - dy ) * 0.25;
            var idx = count << 2;
            curves[ idx ] = qx;
            curves[ idx + 1 ] = qy;
            curves[ idx + 2 ] = dx;
            curves[ idx + 3 ] = dy;
            // update pen
            ax = dx;
            ay = dy;
            count++;
            return;
        }
        var midL1x = ( ax + bx ) * 0.5;
        var midL1y = ( ay + by ) * 0.5;
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
        subdividePerformance( midL1x, midL1y, midL2x, midL2y, splitX, splitY, nextDepth, maxDepth );
        // make sure pen is correct
        ax = splitX;
        ay = splitY;
        subdividePerformance( midR1x, midR1y, midR2x, midR2y, dx, dy, nextDepth, maxDepth );
    }
}
