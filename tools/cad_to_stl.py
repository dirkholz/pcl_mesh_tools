FREECADPATH='/usr/lib/freecad/lib/'
import sys
sys.path.append(FREECADPATH)

import FreeCAD
import Part
import Mesh

import sys
in_fn, out_fn = sys.argv[1], sys.argv[2]

Part.open(in_fn)
o = [ FreeCAD.getDocument("Unnamed").findObjects()[0] ]
Mesh.export(o, out_fn)
