#include "Resovle_NeedleTriangle_face.hpp"


int main() {
    Mesh m;
    // Add the points as vertices
    vertex_descriptor a = m.add_vertex(K::Point_3(0, 26, 0));
    vertex_descriptor b = m.add_vertex(K::Point_3(1, 26, 0));
    vertex_descriptor c = m.add_vertex(K::Point_3(2, 26, 0));

    vertex_descriptor d = m.add_vertex(K::Point_3(0, 25, 0));
    vertex_descriptor e = m.add_vertex(K::Point_3(1, 25, 0));
    vertex_descriptor f = m.add_vertex(K::Point_3(2, 25, 0));

    vertex_descriptor g = m.add_vertex(K::Point_3(0,1,0));
    vertex_descriptor h = m.add_vertex(K::Point_3(1,1,0));
    vertex_descriptor i = m.add_vertex(K::Point_3(2,1,0));

    vertex_descriptor j = m.add_vertex(K::Point_3(0, 0, 0));
    vertex_descriptor k = m.add_vertex(K::Point_3(1, 0, 0));
    vertex_descriptor l = m.add_vertex(K::Point_3(2, 0, 0));

    m.add_face(a,d,e);
    m.add_face(a,e,b);
    m.add_face(b,e,c);
    m.add_face(e,f,c);

    m.add_face(d,g,e);
    m.add_face(e,g,h);
    m.add_face(e,h,f);
    m.add_face(f,h,i);

    m.add_face(g,j,h);
    m.add_face(h,j,k);
    m.add_face(h,k,l);
    m.add_face(h,l,i);



    //输入为逆时针方向底边 halfedge_desciptor:gh 和顶点 vertex_descriptor:e
    halfedge_descriptor gh = m.halfedge(g,h);
    MethodSelector(m,gh);

    halfedge_descriptor fe = m.halfedge(f,e);
    MethodSelector(m,fe);

    halfedge_descriptor ed = m.halfedge(e,d);
    MethodSelector(m,ed);



    std::ofstream out("out.off");
    write_off(out,m);
}