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
    private var writeIndex:Int = 0;

    public function new( size: Int = 2048 ) {
        #if (cpp || cppia || hl || jvm || java)
            this.curves = new haxe.ds.Vector<Float>( size );
        #else
            this.curves = new Array<Float>();
        #end
    }

    public inline function clear():Void {
        writeIndex = 0;
        #if !(cpp || cppia || hl || jvm || java)
            this.curves.resize( 0 );
        #end
    }

    public var length(get, null):Int;
    private inline function get_length():Int { 
        return writeIndex; 
    }

    public inline function addCubicCurve(  p0x: Float, p0y: Float
                                         , p1x: Float, p1y: Float
                                         , p2x: Float, p2y: Float
                                         , p3x: Float, p3y: Float
                                         , mode: OptimizationMode
                                         , errorMargin: Float = 0.5
                                         , maxDepth: Int = 5 ):Int {
        var startIndex = _writeIndex;
        var runAccurate = ( mode == Accurate );
        if( mode == Adaptive ) {
            var dx1 = p1x - p0x;
            var dy1 = p1y - p0y; 
            var dx2 = p3x - p2x;
            var dy2 = p3y - p2y; 
            var crossHandles = dx1 * dy2 - dy1 * dx2;
            var dxBase = p3x - p0x;
            var dyBase = p3y - p0y;
            var side1 = dxBase * (p1y - p0y) - dyBase * (p1x - p0x);
            var side2 = dxBase * (p2y - p0y) - dyBase * (p2x - p0x);
            var isProblematic = ( side1 * side2 < -0.01 ) || (Math.abs( crossHandles ) < 0.05 && ( dx1 * dx1 + dy1 * dy1 > 1.0 ));
            if( isProblematic ){
                runAccurate = true;
                maxDepth += 1;
            }
        }
        if( runAccurate || mode == Accurate ){
            var errorMarginSquared = errorMargin * errorMargin;
            subdivideAccurate( p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y, errorMarginSquared, 0, maxDepth );
        } else {
            subdividePerformance( p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y, 0, 3 );
        }
        return _writeIndex - startIndex; 
    }

    private function subdivideAccurate(  p0x: Float, p0y: Float
                                       , p1x: Float, p1y: Float
                                       , p2x: Float, p2y: Float
                                       , p3x: Float, p3y: Float
                                       , errorMarginSquared: Float
                                       , currentDepth: Int
                                       , maxDepth: Int
    ):Void {
        var dx = p3x - 3.0 * p2x + 3.0 * p1x - p0x;
        var dy = p3y - 3.0 * p2y + 3.0 * p1y - p0y;
        var estimatedErrorSquared = (dx * dx + dy * dy) * 0.009259259259259259;
        if( estimatedErrorSquared <= errorMarginSquared || currentDepth >= maxDepth ){
            var q1x:Float = (3.0 * p1x - p0x + 3.0 * p2x - p3x) * 0.25;
            var q1y:Float = (3.0 * p1y - p0y + 3.0 * p2y - p3y) * 0.25;
            var idx = writeIndex;
            curves[ idx ]     = p0x;
            curves[ idx + 1 ] = p0y;
            curves[ idx + 2 ] = q1x;
            curves[ idx + 3 ] = q1y;
            curves[ idx + 4 ] = p3x;
            curves[ idx + 5 ] = p3y;
            writeIndex = idx + 6;
            return;
        }
        var midL1x = ( p0x + p1x ) * 0.5; 
        var midL1y = ( p0y + p1y ) * 0.5;
        var midMx  = ( p1x + p2x ) * 0.5;
        var midMy  = ( p1y + p2y ) * 0.5;
        var midR2x = ( p2x + p3x ) * 0.5;
        var midR2y = ( p2y + p3y ) * 0.5;
        var midL2x = ( midL1x + midMx ) * 0.5;
        var midL2y = ( midL1y + midMy ) * 0.5;
        var midR1x = ( midMx + midR2x ) * 0.5;
        var midR1y = ( midMy + midR2y ) * 0.5;
        var splitX = ( midL2x + midR1x ) * 0.5;
        var splitY = ( midL2y + midR1y ) * 0.5;
        var nextDepth = currentDepth + 1;
        subdivideAccurate( p0x, p0y, midL1x, midL1y, midL2x, midL2y, splitX, splitY, errorMarginSquared, nextDepth, maxDepth );
        subdivideAccurate( splitX, splitY, midR1x, midR1y, midR2x, midR2y, p3x, p3y, errorMarginSquared, nextDepth, maxDepth );
    }

    private function subdividePerformance(  p0x: Float, p0y: Float
                                          , p1x: Float, p1y: Float
                                          , p2x: Float, p2y: Float
                                          , p3x: Float, p3y:Float
                                          , currentDepth:Int, maxDepth:Int ):Void {
        if( currentDepth >= maxDepth ){
            var q1x = ( 3.0 * p1x - p0x + 3.0 * p2x - p3x ) * 0.25;
            var q1y = ( 3.0 * p1y - p0y + 3.0 * p2y - p3y ) * 0.25;
            var idx = writeIndex;
            curves[ idx ]     = p0x;
            curves[ idx + 1 ] = p0y;
            curves[ idx + 2 ] = q1x;
            curves[ idx + 3 ] = q1y;
            curves[ idx + 4 ] = p3x;
            curves[ idx + 5 ] = p3y;
            writeIndex = idx + 6;
            return;
        }
        var midL1x = ( p0x + p1x ) * 0.5;
        var midL1y = ( p0y + p1y ) * 0.5;
        var midMx  = ( p1x + p2x ) * 0.5;
        var midMy  = ( p1y + p2y ) * 0.5;
        var midR2x = ( p2x + p3x ) * 0.5;
        var midR2y = ( p2y + p3y ) * 0.5;
        var midL2x = ( midL1x + midMx ) * 0.5;
        var midL2y = ( midL1y + midMy ) * 0.5;
        var midR1x = ( midMx + midR2x ) * 0.5;
        var midR1y = ( midMy + midR2y ) * 0.5;
        var splitX = ( midL2x + midR1x ) * 0.5;
        var splitY = ( midL2y + midR1y ) * 0.5;
        var nextDepth = currentDepth + 1;
        subdividePerformance( p0x, p0y, midL1x, midL1y, midL2x, midL2y, splitX, splitY, nextDepth, maxDepth );
        subdividePerformance( splitX, splitY, midR1x, midR1y, midR2x, midR2y, p3x, p3y, nextDepth, maxDepth );
    }
}
