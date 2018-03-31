//
// Created by vincentlv on 17-4-02.
//

#ifndef DBSCAN_H
#define DBSCAN_H

#include <map>
#include <iostream>

template <typename Point>
class DbScan
{
public:    
    explicit DbScan(){}
    // DbScan(const std::vector<Point>* _data,double _eps,int _mnpts):data(_data)
    // {
    //     eps=_eps;
    //     mnpts=_mnpts;
    //     // setInputData(_data);
    // }

    void clear()
    {
        dp.clear();  // clear our dp table
        labels.clear();
        C=0;
        for(int i=0;i<data->size();++i)
        {
            labels[i]=-99;
        }
    }

    // parameter setters
    void setEps(double _eps)
    {
        eps = _eps;
    }
    void setMinPoints(int _mnpts)
    {
        mnpts = _mnpts;
    }

    // parameter getters
    double getEps() const
    {
        return eps;
    }
    int getMinPoints() const
    {
        return mnpts;
    }

    // data setter
    void setInputData(const std::vector<Point>* _data)
    {
        data = _data;
        // data.reserve(_data.size());
        // data.insert(data.end(), _data.begin(), _data.end());
    }

    // getters
    int getClusterSize() const
    {
        return C;
    }
    std::map<int, int> getLabels() const 
    {
        return labels;
    }


    void run()
    {
        clear();

        int data_sz = data->size();
        dp.resize(data_sz*data_sz);
        std::cout << data->size() << std::endl;
        for(int i=0;i<data_sz;++i)
        {
            for(int j=0;j<data_sz;++j)
            {
                DP(i,j) = (i==j) - 1;
                // if(i==j)
                //     DP(i,j)=0;
                // else
                //     DP(i,j)=-1;
            }
        }
        for(int i=0;i<data_sz;++i)
        {
            if(!isVisited(i))
            {
                std::vector<int> neighbours = regionQuery(i);
                if(neighbours.size()<mnpts)
                {
                    labels[i]=-1;//noise
                }
                else
                {
                    ++C;
                    expandCluster(i,neighbours);
                }
            }
        }
    }

    std::vector<std::vector<Point> > getClusters() const
    {
        std::vector<std::vector<Point> > ret;
        int cluster_sz = getClusterSize();
        if (cluster_sz > 0)
        {
            ret.reserve(cluster_sz);
            for(int i=0;i< cluster_sz;++i)
            {
                ret.push_back(std::vector<Point>{});
                for(int j=0;j<data->size();++j)
                {
                    if(labels.at(j)==i)
                    {
                        ret[ret.size()-1].push_back((*data)[j]);
                    }
                }
            }
        }
        return ret;
    }

private:
    void expandCluster(int p, const std::vector<int>& neighbours)
    {
        int c_idx = C - 1;  // zero indexed
        labels[p]=c_idx;
        for(int i=0;i<neighbours.size();++i)
        {
            if(!isVisited(neighbours[i]))
            {
                labels[neighbours[i]]=c_idx;
                std::vector<int> neighbours_p = regionQuery(neighbours[i]);
                if (neighbours_p.size() >= mnpts)
                {
                    expandCluster(neighbours[i],neighbours_p);
                }
            }
        }
    }

    bool isVisited(int i) const
    {
        return labels.at(i)!=-99;
    }

    std::vector<int> regionQuery(int p)
    {
        std::vector<int> res;
        for(int i=0;i<data->size();++i)
        {
            if(distanceFunc(p,i)<=eps)
            {
                res.push_back(i);
            }
        }
        return res;
    }

    inline double dist2d(const Point& a, const Point& b)
    {
        return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
    }

    inline double& DP(int i, int j)
    {
        return dp[(data->size()*i)+j];
    }

    double distanceFunc(int ai,int bi)
    {
        if(DP(ai,bi)!=-1)
            return DP(ai,bi);
        
        double minDist = 9999999;

        minDist = std::min(minDist,dist2d((*data)[ai], (*data)[bi]));

        DP(ai,bi)=minDist;
        DP(bi,ai)=minDist;
        return DP(ai,bi);
    }


    // variables

    std::map<int, int> labels;
    const std::vector<Point>* data;
    int C = 0;
    double eps = 15;
    int mnpts = 2;  // min points per cluster

    //memoization table in case of complex dist functions
    std::vector<double> dp;

};


#endif // DBSCAN_H
