#include <cmath>
// Controller class definition and get_v_next implementation
class Monitor;

class Controller {
public:
    Vec pos_cur, v_cur, pos_tar;
    double r, v_max;
    int id;
    Monitor* monitor;

    Controller(const Vec& _pos_tar, double _v_max, double _r, int _id, Monitor* _monitor)
        : pos_cur(), v_cur(), pos_tar(_pos_tar), r(_r), v_max(_v_max), id(_id), monitor(_monitor) {}

    void set_pos_cur(const Vec& p) { pos_cur = p; }
    void set_v_cur(const Vec& v) { v_cur = v; }

    Vec get_v_next() {
        double tau = 0.1;

        Vec to_tar = pos_tar - pos_cur;
        double dist2 = to_tar.dot(to_tar);
        if (!(dist2 < 1e300)) return Vec(0.0, 0.0);
        if (dist2 <= 1e-12) return Vec(0.0, 0.0);
        double dist = std::sqrt(dist2);
        if (!(dist > 0.0)) return Vec(0.0, 0.0);
        Vec dir = to_tar * (1.0 / dist);

        double sp = dist / tau;
        double desired_speed = (sp < v_max ? sp : v_max);
        if (!(desired_speed >= 0.0)) desired_speed = 0.0;

        auto min_dist2_interval = [&](const Vec &r0, const Vec &vrel, double T) -> double {
            double v2 = vrel.dot(vrel);
            if (v2 <= 1e-12) return r0.dot(r0);
            double tstar = - r0.dot(vrel) / v2;
            if (tstar < 0.0) tstar = 0.0;
            if (tstar > T) tstar = T;
            Vec d = r0 + vrel * tstar;
            return d.dot(d);
        };
        auto is_safe = [&](const Vec &vel) -> bool {
            int N = monitor->get_robot_number();
            for (int j = 0; j < N; ++j) {
                if (j == id) continue;
                Vec pj = monitor->get_pos_cur(j);
                Vec vj = monitor->get_v_cur(j);
                double Rsum = r + monitor->get_r(j);
                Vec r0 = pos_cur - pj;
                Vec vrel = vel - vj;
                double md2 = min_dist2_interval(r0, vrel, tau);
                double thresh = Rsum - 1e-9;
                if (md2 < thresh * thresh) return false;
            }
            return true;
        };

        Vec v = dir * desired_speed;
        if (is_safe(v)) return v;

        const double scales[] = {1.0, 0.9, 0.7, 0.5, 0.3, 0.15, 0.0};
        for (double s : scales) {
            Vec base = dir * (desired_speed * s);
            if (is_safe(base)) return base;
            Vec neg = base * (-1.0);
            if (is_safe(neg)) return neg;
        }
        return Vec(0.0, 0.0);
    }
};
