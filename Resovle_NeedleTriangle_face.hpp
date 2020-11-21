//
// Created by Cory on 2020/11/18.
//

#ifndef MYPROJECT_RESOVLE_NEEDLETRIANGLE_FACE_HPP
#define MYPROJECT_RESOVLE_NEEDLETRIANGLE_FACE_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <fstream>
#include <iostream>
#include <CGAL/boost/graph/Euler_operations.h>
#include <unordered_set>
#include <map>
#include <vector>
typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
typedef Mesh::Halfedge_index halfedge_descriptor;


#define resolution 1.0  //全局的分辨率精度控制


#endif //MYPROJECT_RESOVLE_NEEDLETRIANGLE_FACE_HPP

const double edge_length = resolution*2;  //边长的阈值，超过该值都需要分割

//输入三角形需要严格按照输入顺序
std::unordered_set<face_descriptor> is_face_handled;    //看是否输入半边hd对应的面f被处理过，根据是否能find到分别以不同的函数处理
std::map<face_descriptor,char> f_to_side;          //如果上面的unordered_set返回的结果是处理过，根据map查看左右的情况


double Compute_dis(const Point& A,const Point& B){
    K::Compute_squared_distance_3 squared_distance;
    return sqrt(squared_distance(A,B));
}

int Compute_count(Mesh& m,halfedge_descriptor hd){
    vertex_descriptor begin = m.source(hd);
    vertex_descriptor end = m.target(hd);
    double length = Compute_dis(m.point(begin),m.point(end));
    int cnt;
    cnt = int(length/resolution);
    return cnt+1;   //边界条件，同时避免除0
}

//输入为逆时针方向底边 halfedge_desciptor:gh 和顶点 vertex_descriptor:e
//需要处理count


//所有的三角形点的定义都基于如下图形
//       c
//      /\
//     /  \
//    /    \
//   /      \
//  a--------b
void Empty_triangle(Mesh& m,halfedge_descriptor hd)
{
    face_descriptor f = m.face(hd);
    is_face_handled.insert(f);              //标记f为已处理

    halfedge_descriptor right = m.opposite(m.next(hd));
    halfedge_descriptor left = m.opposite(m.prev(hd));      //左右半边各自对应的opposite

    face_descriptor fright = m.face(right);             //左右半边对边对应的面f
    face_descriptor fleft = m.face(left);

    f_to_side[fright] = 'r';                    //标记左右面为已处理，并哈希映射
    is_face_handled.insert(fright);
    f_to_side[fleft] = 'l';
    is_face_handled.insert(fleft);

    vertex_descriptor a,b,c;
    a = m.source(hd);
    b = m.target(hd);
    c = m.source(m.prev(hd));

    K::Point_3 A = m.point(a);
    K::Point_3 B = m.point(b);
    K::Point_3 C = m.point(c);

    int cnt = std::max(Compute_count(m,m.halfedge(a,c)),Compute_count(m,m.halfedge(b,c)));   //计算点分裂次数

    //右边的起始边ba
    halfedge_descriptor hdr = m.halfedge(b,a);
    //左边的起始边cb
    halfedge_descriptor hdl = m.halfedge(c,b);

    std::vector<halfedge_descriptor> vec_hdr;
    std::vector<halfedge_descriptor> vec_hdl;
    vec_hdr.reserve(cnt);
    vec_hdl.reserve(cnt);

    for(size_t i = 1; i < cnt; ++i){
        hdl = CGAL::Euler::split_vertex(m.opposite(hdl),m.halfedge(a,m.source(hdl)),m);
        m.point(m.source(hdl)) = C+(A-C)*i/cnt;
        //todo:保存每次分裂后的半边hdl,用于分裂面
        vec_hdl.push_back(hdl);
        hdr = CGAL::Euler::split_vertex(m.opposite(hdr),m.halfedge(c,m.source(hdr)),m);
        m.point(m.source(hdr)) = B+(C-B)*i/cnt;  //根据ratio来算
        //todo:保存每次分裂后的半边hdr,用于分裂面
        vec_hdr.push_back(hdr);
    }

    auto itr = vec_hdr.begin();
    auto itl = vec_hdl.rbegin();
    while(itr != vec_hdr.end()){
        CGAL::Euler::split_face(m.opposite(*itr),m.opposite(*itl),m);
        ++itr,++itl;
    }

//    CGAL::Euler::split_face(m.opposite(hr1),m.opposite(hl2),m);
//    CGAL::Euler::split_face(m.opposite(hr2),m.opposite(hl1),m);



//    halfedge_descriptor hr2 = CGAL::Euler::split_vertex(m.opposite(hr1),m.halfedge(c,m.source(hr1)),m);
//    m.point(m.source(hr2)) = K::Point_3(1,3,0);


//    halfedge_descriptor ac = m.opposite(ca);
//    halfedge_descriptor tmp = m.prev(ca);

//
//    m.point(m.source(hl1)) = K::Point_3(0.5,3,0);
//
//    halfedge_descriptor hl2 = CGAL::Euler::split_vertex(m.opposite(hl1),m.halfedge(a,m.source(hl1)),m);
//    m.point(m.source(hl2)) = K::Point_3(0.25,2,0);



}

void handled_right_wing_triangle(Mesh& m,halfedge_descriptor hd){
    vertex_descriptor a,b,c;
    a = m.source(hd);
    b = m.target(hd);
    c = m.source(m.prev(hd));

    K::Point_3 A = m.point(a);
    K::Point_3 B = m.point(b);
    K::Point_3 C = m.point(c);

    halfedge_descriptor left = m.opposite(m.prev(hd));
    face_descriptor fleft = m.face(left);
    f_to_side[fleft] = 'l';
    is_face_handled.insert(fleft);

    halfedge_descriptor ca = m.halfedge(c,a);
    halfedge_descriptor begin = m.prev(ca);
    halfedge_descriptor hdl = m.opposite(begin);

    std::vector<halfedge_descriptor> vec_hdr;

    halfedge_descriptor temp = ca;
    while(m.prev(temp) != ca){
        vec_hdr.push_back(m.prev(temp));
        temp = m.prev(temp);
    }
    vec_hdr.pop_back(); //很关键的1次pop用以排除ab(下标从1开始)以及右边上的最后一个半边(pop)

    size_t cnt = vec_hdr.size();
    std::vector<halfedge_descriptor> vec_hdl;
    vec_hdl.reserve(cnt);

    for(int i = 1; i < cnt; ++i){
        hdl = CGAL::Euler::split_vertex(m.opposite(hdl),m.halfedge(a,m.source(hdl)),m);
        m.point(m.source(hdl)) = C+(A-C)*i/cnt;
        //todo:保存每次分裂后的半边hdl,用于分裂面
        vec_hdl.push_back(hdl);
    }

    auto itr = vec_hdr.rbegin();
    auto itl = vec_hdl.rbegin();
    while(itr != vec_hdr.rend()-1){
        CGAL::Euler::split_face(*itr,m.opposite(*itl),m);
        ++itr,++itl;
    }

}

void handled_left_wing_triangle(Mesh& m,halfedge_descriptor hd){
    vertex_descriptor a,b,c;
    a = m.source(hd);
    b = m.target(hd);
    c = m.target(m.next(hd));

    K::Point_3 A = m.point(a);
    K::Point_3 B = m.point(b);
    K::Point_3 C = m.point(c);

    halfedge_descriptor right = m.opposite(m.next(hd));
    face_descriptor fright = m.face(right);
    f_to_side[fright] = 'r';
    is_face_handled.insert(fright);

    halfedge_descriptor bc = m.halfedge(b,c);
    halfedge_descriptor hdr = m.halfedge(b,a);

    std::vector<halfedge_descriptor> vec_hdl;
    halfedge_descriptor temp = bc;
    while(m.next(temp) != bc){
        vec_hdl.push_back(m.next(temp));
        temp = m.next(temp);
    }
    vec_hdl.pop_back(); //很关键的1次pop用以排除ab(pop)以及右边上的最后一个半边(下标从1开始) note：这个和right wing是完全相反的
    vec_hdl.pop_back();

    size_t cnt = vec_hdl.size();

    std::vector<halfedge_descriptor> vec_hdr;
    vec_hdl.reserve(cnt);

    for(int i = 1; i < cnt; ++i){
        hdr = CGAL::Euler::split_vertex(m.opposite(hdr),m.halfedge(c,m.source(hdr)),m);
        m.point(m.source(hdr)) = B+(C-B)*i/cnt;  //根据ratio来算
        vec_hdr.push_back(hdr);
    }

    auto itr = vec_hdr.begin();
    auto itl = vec_hdl.rbegin();
    while(itr != vec_hdr.end()){
        CGAL::Euler::split_face(m.opposite(*itr),*itl,m);
        ++itr,++itl;
    }

}

void MethodSelector(Mesh& m,halfedge_descriptor hd){
    face_descriptor f = m.face(hd);
    if(is_face_handled.find(f)==is_face_handled.end()){
        Empty_triangle(m,hd);
    }else{
        if(f_to_side[f] == 'l'){
            handled_left_wing_triangle(m,hd);
        }
        else if(f_to_side[f] == 'r')
        {
            handled_right_wing_triangle(m,hd);
        }
        else{
            std::cout<<"nothing happens"<<std::endl;
        }
    }
}
