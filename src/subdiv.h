#ifndef SUBDIV_H
#define SUBDIV_H

#include "util.h"


class Subdiv
{
public:
    /** Subdiv point location cases */
    enum { PTLOC_ERROR        = -2, //!< Point location error
           PTLOC_OUTSIDE_RECT = -1, //!< Point outside the subdivision bounding rect
           PTLOC_INSIDE       = 0, //!< Point inside some facet
           PTLOC_VERTEX       = 1, //!< Point coincides with one of the subdivision vertices
           PTLOC_ON_EDGE      = 2  //!< Point on some edge
         };

    /** Subdiv edge type navigation (see: getEdge()) */
    enum { NEXT_AROUND_ORG   = 0x00,
           NEXT_AROUND_DST   = 0x22,
           PREV_AROUND_ORG   = 0x11,
           PREV_AROUND_DST   = 0x33,
           NEXT_AROUND_LEFT  = 0x13,
           NEXT_AROUND_RIGHT = 0x31,
           PREV_AROUND_LEFT  = 0x20,
           PREV_AROUND_RIGHT = 0x02
         };

    Subdiv();

    Subdiv(Rect rect);

    void initDelaunay(Rect rect);

    int insert(Point2f pt);

    void insert(const std::vector<Point2f>& ptvec);

    int locate(Point2f pt, int& edge, int& vertex);

    int findNearest(Point2f pt, Point2f* nearestPt = 0);

    void getEdgeList(std::vector<Vec4f>& edgeList) const;

    void getEdgeIdxList(std::vector<int>& edgeIdxList) const;
    int getEdgeVtx1(int edgeidx, Point2f* pt=0);
    int getEdgeVtx2(int edgeidx, Point2f* pt=0);

    void getLeadingEdgeList(std::vector<int>& leadingEdgeList) const;

    void getTriangleList(std::vector<Vec6f>& triangleList) const;

    void getVoronoiFacetList(const std::vector<int>& idx, std::vector<std::vector<Point2f> >& facetList,
                                     std::vector<Point2f>& facetCenters);

    Point2f getVertex(int vertex, int* firstEdge = 0) const;

    int getEdge( int edge, int nextEdgeType ) const;

    int nextEdge(int edge) const;

    int rotateEdge(int edge, int rotate) const;
    int symEdge(int edge) const;

    int edgeOrg(int edge, Point2f* orgpt = 0) const;

    int edgeDst(int edge, Point2f* dstpt = 0) const;

protected:
    int newEdge();
    void deleteEdge(int edge);
    int newPoint(Point2f pt, bool isvirtual, int firstEdge = 0);
    void deletePoint(int vtx);
    void setEdgePoints( int edge, int orgPt, int dstPt );
    void splice( int edgeA, int edgeB );
    int connectEdges( int edgeA, int edgeB );
    void swapEdges( int edge );
    int isRightOf(Point2f pt, int edge) const;
    void calcVoronoi();
    void clearVoronoi();
    void checkSubdiv() const;

    struct Vertex
    {
        Vertex();
        Vertex(Point2f pt, bool _isvirtual, int _firstEdge=0);
        bool isvirtual() const;
        bool isfree() const;

        int firstEdge;
        int type;
        Point2f pt;
    };

    struct QuadEdge
    {
        QuadEdge();
        QuadEdge(int edgeidx);
        bool isfree() const;

        int next[4];
        int pt[4];
    };

    //! All of the vertices
    std::vector<Vertex> vtx;
    //! All of the edges
    std::vector<QuadEdge> qedges;
    int freeQEdge;
    int freePoint;
    bool validGeometry;

    int recentEdge;
    //! Top left corner of the bounding rect
    Point2f topLeft;
    //! Bottom right corner of the bounding rect
    Point2f bottomRight;
};



#endif
