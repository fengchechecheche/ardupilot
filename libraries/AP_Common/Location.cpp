/*
 * Location.cpp
 */

#include "Location.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>

/// constructors
Location::Location()
{
    zero();
}

const Location definitely_zero{};
bool Location::is_zero(void) const
{
    return !memcmp(this, &definitely_zero, sizeof(*this));
}

void Location::zero(void)
{
    memset(this, 0, sizeof(*this));
}

// Construct location using position (NEU) from ekf_origin for the given altitude frame
Location::Location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, AltFrame frame)
{
    zero();
    lat = latitude;
    lng = longitude;
    set_alt_cm(alt_in_cm, frame);
}

Location::Location(const Vector3f &ekf_offset_neu, AltFrame frame)
{
    zero();

    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, frame);

    // calculate lat, lon
    Location ekf_origin;
    if (AP::ahrs().get_origin(ekf_origin)) {
        lat = ekf_origin.lat;
        lng = ekf_origin.lng;
        offset(ekf_offset_neu.x * 0.01, ekf_offset_neu.y * 0.01);
    }
}

Location::Location(const Vector3d &ekf_offset_neu, AltFrame frame)
{
    zero();

    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, frame);

    // calculate lat, lon
    Location ekf_origin;
    if (AP::ahrs().get_origin(ekf_origin)) {
        lat = ekf_origin.lat;
        lng = ekf_origin.lng;
        offset(ekf_offset_neu.x * 0.01, ekf_offset_neu.y * 0.01);
    }
}

void Location::set_alt_cm(int32_t alt_cm, AltFrame frame)
{
    alt = alt_cm;
    relative_alt = false;
    terrain_alt = false;
    origin_alt = false;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            // do nothing
            break;
        case AltFrame::ABOVE_HOME:
            relative_alt = true;
            break;
        case AltFrame::ABOVE_ORIGIN:
            origin_alt = true;
            break;
        case AltFrame::ABOVE_TERRAIN:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            relative_alt = true;
            terrain_alt = true;
            break;
    }
}

// converts altitude to new frame
bool Location::change_alt_frame(AltFrame desired_frame)
{
    int32_t new_alt_cm;
    if (!get_alt_cm(desired_frame, new_alt_cm)) {
        return false;
    }
    set_alt_cm(new_alt_cm, desired_frame);
    return true;
}

// get altitude frame
Location::AltFrame Location::get_alt_frame() const
{
    if (terrain_alt) {
        return AltFrame::ABOVE_TERRAIN;
    }
    if (origin_alt) {
        return AltFrame::ABOVE_ORIGIN;
    }
    if (relative_alt) {
        return AltFrame::ABOVE_HOME;
    }
    return AltFrame::ABSOLUTE;
}

/// get altitude in desired frame
bool Location::get_alt_cm(AltFrame desired_frame, int32_t &ret_alt_cm) const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!initialised()) {
        AP_HAL::panic("Should not be called on invalid location: Location cannot be (0, 0, 0)");
    }
#endif
    Location::AltFrame frame = get_alt_frame();

    // shortcut if desired and underlying frame are the same
    if (desired_frame == frame) {
        ret_alt_cm = alt;
        return true;
    }

    // check for terrain altitude
    float alt_terr_cm = 0;
    if (frame == AltFrame::ABOVE_TERRAIN || desired_frame == AltFrame::ABOVE_TERRAIN) {
#if AP_TERRAIN_AVAILABLE
        AP_Terrain *terrain = AP::terrain();
        if (terrain == nullptr) {
            return false;
        }
        if (!terrain->height_amsl(*this, alt_terr_cm)) {
            return false;
        }
        // convert terrain alt to cm
        alt_terr_cm *= 100.0f;
#else
        return false;
#endif
    }

    // convert alt to absolute
    int32_t alt_abs = 0;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            alt_abs = alt;
            break;
        case AltFrame::ABOVE_HOME:
            if (!AP::ahrs().home_is_set()) {
                return false;
            }
            alt_abs = alt + AP::ahrs().get_home().alt;
            break;
        case AltFrame::ABOVE_ORIGIN:
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (!AP::ahrs().get_origin(ekf_origin)) {
                    return false;
                }
                alt_abs = alt + ekf_origin.alt;
            }
            break;
        case AltFrame::ABOVE_TERRAIN:
            alt_abs = alt + alt_terr_cm;
            break;
    }

    // convert absolute to desired frame
    switch (desired_frame) {
        case AltFrame::ABSOLUTE:
            ret_alt_cm = alt_abs;
            return true;
        case AltFrame::ABOVE_HOME:
            if (!AP::ahrs().home_is_set()) {
                return false;
            }
            ret_alt_cm = alt_abs - AP::ahrs().get_home().alt;
            return true;
        case AltFrame::ABOVE_ORIGIN:
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (!AP::ahrs().get_origin(ekf_origin)) {
                    return false;
                }
                ret_alt_cm = alt_abs - ekf_origin.alt;
                return true;
            }
        case AltFrame::ABOVE_TERRAIN:
            ret_alt_cm = alt_abs - alt_terr_cm;
            return true;
    }
    return false;  // LCOV_EXCL_LINE  - not reachable
}

bool Location::get_vector_xy_from_origin_NE(Vector2f &vec_ne) const
{
    Location ekf_origin;
    if (!AP::ahrs().get_origin(ekf_origin)) {
        return false;
    }
    vec_ne.x = (lat-ekf_origin.lat) * LATLON_TO_CM;
    vec_ne.y = diff_longitude(lng,ekf_origin.lng) * LATLON_TO_CM * longitude_scale((lat+ekf_origin.lat)/2);
    return true;
}

bool Location::get_vector_from_origin_NEU(Vector3f &vec_neu) const
{
    // convert lat, lon
    Vector2f vec_ne;
    if (!get_vector_xy_from_origin_NE(vec_ne)) {
        return false;
    }
    vec_neu.x = vec_ne.x;
    vec_neu.y = vec_ne.y;

    // convert altitude
    int32_t alt_above_origin_cm = 0;
    if (!get_alt_cm(AltFrame::ABOVE_ORIGIN, alt_above_origin_cm)) {
        return false;
    }
    vec_neu.z = alt_above_origin_cm;

    return true;
}

// return distance in meters between two locations
// 它用于计算两个地点之间的距离（以米为单位）。
// 这是 Location 类的一个成员函数，用于计算当前对象（即 *this）与另一个 Location 对象 loc2 之间的距离。
// 函数接受一个对 Location 类型的常量引用作为参数，即 loc2。由于是常量引用，因此函数内部不会修改 loc2 的内容。
// 成员函数末尾的 const 表示这个函数不会修改调用它的对象的状态（即 *this）。
ftype Location::get_distance(const struct Location &loc2) const
{
    // 计算纬度差（dlat），其中 loc2.lat 是第二个位置的纬度，lat 是当前对象的纬度。
    ftype dlat = (ftype)(loc2.lat - lat);
    // 计算经度差（dlng），这里涉及两个函数调用：
    // diff_longitude(loc2.lng, lng)：这个函数用来计算两个经度之间的差值，并考虑地球是球体的事实（即经度线在地球两极处会相交）。
    // longitude_scale((lat+loc2.lat)/2)：这个函数可能是用来计算给定纬度附近的经度线的实际长度。
    // 由于地球的形状，不同纬度上的经度线长度不同，因此需要进行缩放。
    ftype dlng = ((ftype)diff_longitude(loc2.lng,lng)) * longitude_scale((lat+loc2.lat)/2);

    // norm(dlat, dlng)：这个函数可能是用来计算从原点到点 (dlat, dlng) 的欧几里得距离。
    // 在这里，它代表在二维平面上纬度差和经度差对应的“直线”距离。
    // LOCATION_SCALING_FACTOR：这是一个常数，用于将二维平面上的距离转换为实际地球表面的距离。
    // 由于地球是球体，直接计算经纬度差值得到的距离并不准确，因此需要乘以这个缩放因子进行修正。
    return norm(dlat, dlng) * LOCATION_SCALING_FACTOR;
}

// return the altitude difference in meters taking into account alt frame.
bool Location::get_alt_distance(const struct Location &loc2, ftype &distance) const
{
    int32_t alt1, alt2;
    if (!get_alt_cm(AltFrame::ABSOLUTE, alt1) || !loc2.get_alt_cm(AltFrame::ABSOLUTE, alt2)) {
        return false;
    }
    distance = (alt1 - alt2) * 0.01;
    return true;
}

/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
  这个函数计算从当前对象（*this）到另一个 Location 对象（loc2）在东北平面（即不考虑地球曲率）上的距离，
  并返回一个 Vector2f 类型的向量，该向量表示北（N）和东（E）方向上的分量。
 */
// Vector2f：返回值的类型，表示一个二维向量，通常包含两个浮点数分量。
// Location::get_distance_NE：这是 Location 类的一个成员函数。
// const Location &loc2：输入参数，表示另一个 Location 对象。
// const：表示这个函数不会修改调用它的对象的状态。
Vector2f Location::get_distance_NE(const Location &loc2) const
{
    // loc2.lat - lat：计算 loc2 和当前对象在纬度上的差异。
    // LOCATION_SCALING_FACTOR：一个缩放因子，用于将纬度或经度差异转换为实际距离。
    // 这个因子的具体值取决于地球模型的细节和所使用的单位（例如，米）。
    // diff_longitude(loc2.lng,lng)：调用一个名为 diff_longitude 的函数，用于计算 loc2 和当前对象在经度上的差异。
    // 这个函数可能考虑了地球曲率对经度线长度的影响。
    // LOCATION_SCALING_FACTOR：同样用于将经度差异转换为实际距离。
    // ongitude_scale((loc2.lat+lat)/2)：调用 longitude_scale 函数，传入 loc2 和当前对象纬度的平均值作为参数。
    // 这个函数返回给定纬度下的经度缩放因子，用于修正由于地球曲率导致的经度线长度变化。

    // 返回结果：return Vector2f(北方向分量, 东方向分量);
    // 函数最后返回一个 Vector2f 类型的向量，该向量的 x 分量表示北方向上的距离，y 分量表示东方向上的距离。
    return Vector2f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((loc2.lat+lat)/2));
}

// return the distance in meters in North/East/Down plane as a N/E/D vector to loc2, NOT CONSIDERING ALT FRAME!
Vector3f Location::get_distance_NED(const Location &loc2) const
{
    return Vector3f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((lat+loc2.lat)/2),
                    (alt - loc2.alt) * 0.01);
}

// return the distance in meters in North/East/Down plane as a N/E/D vector to loc2
Vector3d Location::get_distance_NED_double(const Location &loc2) const
{
    return Vector3d((loc2.lat - lat) * double(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((lat+loc2.lat)/2),
                    (alt - loc2.alt) * 0.01);
}

Vector2d Location::get_distance_NE_double(const Location &loc2) const
{
    return Vector2d((loc2.lat - lat) * double(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * double(LOCATION_SCALING_FACTOR) * longitude_scale((lat+loc2.lat)/2));
}

Vector2F Location::get_distance_NE_ftype(const Location &loc2) const
{
    return Vector2F((loc2.lat - lat) * ftype(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * ftype(LOCATION_SCALING_FACTOR) * longitude_scale((lat+loc2.lat)/2));
}

// extrapolate latitude/longitude given distances (in meters) north and east
void Location::offset_latlng(int32_t &lat, int32_t &lng, ftype ofs_north, ftype ofs_east)
{
    const int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
    const int64_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat+dlat/2);
    lat += dlat;
    lat = limit_lattitude(lat);
    lng = wrap_longitude(dlng+lng);
}

// extrapolate latitude/longitude given distances (in meters) north and east
void Location::offset(ftype ofs_north, ftype ofs_east)
{
    offset_latlng(lat, lng, ofs_north, ofs_east);
}

/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void Location::offset_bearing(ftype bearing_deg, ftype distance)
{
    const ftype ofs_north = cosF(radians(bearing_deg)) * distance;
    const ftype ofs_east  = sinF(radians(bearing_deg)) * distance;
    offset(ofs_north, ofs_east);
}

// extrapolate latitude/longitude given bearing, pitch and distance
void Location::offset_bearing_and_pitch(ftype bearing_deg, ftype pitch_deg, ftype distance)
{
    const ftype ofs_north =  cosF(radians(pitch_deg)) * cosF(radians(bearing_deg)) * distance;
    const ftype ofs_east  =  cosF(radians(pitch_deg)) * sinF(radians(bearing_deg)) * distance;
    offset(ofs_north, ofs_east);
    const int32_t dalt =  sinF(radians(pitch_deg)) * distance *100.0f;
    alt += dalt; 
}


/*
 * 该函数的作用是计算给定纬度下的经度缩放因子。
 * 这个缩放因子通常用于计算地球上两点之间沿经度线的实际距离，
 * 因为地球的形状是椭球体（近似于球体），所以不同纬度处的经度线长度并不相同。
 *
 * 这个函数基于给定的纬度值计算经度缩放因子。
 * 由于地球的形状，纬度越高，沿经度线的实际距离越短，因此需要一个缩放因子来准确计算距离。
 * 这个函数通过计算给定纬度处的余弦值来得到这个缩放因子，并确保它不会太小。
 * */
//ftype：返回值的类型，可能是浮点数类型，如 float 或 double，具体取决于 ftype 的定义。
// Location::longitude_scale：这是 Location 类的一个成员函数。
// int32_t lat：输入参数，表示纬度值，以微度（microdegrees）为单位。
ftype Location::longitude_scale(int32_t lat)
{
    // lat * (1.0e-7 * DEG_TO_RAD)：首先将纬度从微度转换为弧度。
    // 1.0e-7 是将微度转换为度的转换因子，DEG_TO_RAD 是将度转换为弧度的转换因子。
    // cosF(...)：计算给定弧度值的余弦值。
    // scale：存储计算得到的余弦值，这个值将用作缩放因子。
    ftype scale = cosF(lat * (1.0e-7 * DEG_TO_RAD));

    // MAX(scale, 0.01)：返回 scale 和 0.01 中的较大值。
    // 这是为了确保缩放因子不会太小，从而避免在计算距离时出现极端或无效的结果。
    return MAX(scale, 0.01);
}

/*
 * convert invalid waypoint with useful data. return true if location changed
 */
bool Location::sanitize(const Location &defaultLoc)
{
    bool has_changed = false;
    // convert lat/lng=0 to mean current point
    if (lat == 0 && lng == 0) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    // convert relative alt=0 to mean current alt
    if (alt == 0 && relative_alt) {
        relative_alt = false;
        alt = defaultLoc.alt;
        has_changed = true;
    }

    // limit lat/lng to appropriate ranges
    if (!check_latlng()) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    return has_changed;
}

// make sure we know what size the Location object is:
assert_storage_size<Location, 16> _assert_storage_size_Location;


// return bearing in radians from location to loc2, return is 0 to 2*Pi
ftype Location::get_bearing(const struct Location &loc2) const
{
    const int32_t off_x = diff_longitude(loc2.lng,lng);
    const int32_t off_y = (loc2.lat - lat) / loc2.longitude_scale((lat+loc2.lat)/2);
    ftype bearing = (M_PI*0.5) + atan2F(-off_y, off_x);
    if (bearing < 0) {
        bearing += 2*M_PI;
    }
    return bearing;
}

/*
  return true if lat and lng match. Ignores altitude and options
 */
bool Location::same_latlon_as(const Location &loc2) const
{
    return (lat == loc2.lat) && (lng == loc2.lng);
}

// return true when lat and lng are within range
bool Location::check_latlng() const
{
    return check_lat(lat) && check_lng(lng);
}

// see if location is past a line perpendicular to
// the line between point1 and point2 and passing through point2.
// If point1 is our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool Location::past_interval_finish_line(const Location &point1, const Location &point2) const
{
    return this->line_path_proportion(point1, point2) >= 1.0f;
}

/*
  return the proportion we are along the path from point1 to
  point2, along a line parallel to point1<->point2.
  这个函数计算当前对象（*this，即飞机当前位置）沿着从 point1 到 point2 的路径上的比例。
  这个比例是相对于一条与 point1 到 point2 方向平行的直线来计算的。

  This will be more than 1 if we have passed point2
  如果当前对象已经超过了 point2，则这个比例会大于1。
 */
float Location::line_path_proportion(const Location &point1, const Location &point2) const
{
    // 调用 point1 的 get_distance_NE 方法来计算 point1 到 point2 的东北方向（或可能是其他方向的）向量，并存储在 vec1 中。
    const Vector2f vec1 = point1.get_distance_NE(point2);
    // 调用 point1 的 get_distance_NE 方法来计算 point1 到当前对象（*this，即飞机当前位置）的东北方向向量，并存储在 vec2 中。
    const Vector2f vec2 = point1.get_distance_NE(*this);
    // 计算 vec1 的 x 和 y 分量的平方,并存储到 dsquared 中。
    const ftype dsquared = sq(vec1.x) + sq(vec1.y);

    // 检查点是否接近
    // 如果 dsquared（即 vec1 的长度的平方）小于一个很小的值（0.001f），
    // 则认为 point1 和 point2 非常接近。在这种情况下，函数返回 1.0f。
    if (dsquared < 0.001f) {
        // the two points are very close together
        return 1.0f;
    }

    // 计算并返回比例
    // 计算 vec1 和 vec2 的点积
    // 将点积除以 vec1 的长度的平方，得到当前对象沿着从 point1 到 point2 的路径的比例。
    return (vec1 * vec2) / dsquared;
}

/*
  wrap longitude for -180e7 to 180e7
 */
int32_t Location::wrap_longitude(int64_t lon)
{
    if (lon > 1800000000L) {
        lon = int32_t(lon-3600000000LL);
    } else if (lon < -1800000000L) {
        lon = int32_t(lon+3600000000LL);
    }
    return int32_t(lon);
}

/*
  get lon1-lon2, wrapping at -180e7 to 180e7
  这段代码是Location类的一个成员函数，名为diff_longitude，用于计算两个经度之间的差值，同时处理经度环绕的问题。
  在地理坐标系中，经度从-180度（或-180e7微度）到180度（或180e7微度）循环，
  这意味着-179度和180度之间的差值应该是1度，而不是359度。这个函数就是用来处理这种环绕问题的。

  这个函数确保了无论两个经度值如何分布（例如，一个在正数区域，另一个在负数区域），它都能返回正确的差值。
 */
int32_t Location::diff_longitude(int32_t lon1, int32_t lon2)
{
    // 1.检查经度的符号：
    // 这里使用了位运算来检查两个经度值的符号是否相同。
    // 0x80000000是一个32位整数，其最高位（符号位）为1，其余位为0。
    // 通过&操作，我们可以得到lon1和lon2的符号位。
    // 如果它们的符号位相同（即它们都是正数或都是负数），那么我们可以直接相减得到它们的差值。
    if ((lon1 & 0x80000000) == (lon2 & 0x80000000)) {
        // common case of same sign
        return lon1 - lon2;
    }

    // 2.处理经度环绕：
    // 如果两个经度的符号不同，说明它们跨越了-180度到180度的边界。
    // 在这种情况下，我们需要调整差值以确保其在-180到180的范围内。
    // 首先，我们将两个经度转换为int64_t类型，以防止在计算差值时发生溢出。
    // 然后，我们检查差值是否大于180度或小于-180度。
    // 如果是，我们相应地加上或减去360度，以将差值调整到正确的范围内。
    int64_t dlon = int64_t(lon1)-int64_t(lon2);
    if (dlon > 1800000000LL) {
        dlon -= 3600000000LL;
    } else if (dlon < -1800000000LL) {
        dlon += 3600000000LL;
    }

    // 最后，我们将调整后的差值转换回int32_t类型，并返回。
    return int32_t(dlon);
}

/*
  limit lattitude to -90e7 to 90e7
 */
int32_t Location::limit_lattitude(int32_t lat)
{
    if (lat > 900000000L) {
        lat = 1800000000LL - lat;
    } else if (lat < -900000000L) {
        lat = -(1800000000LL + lat);
    }
    return lat;
}

// update altitude and alt-frame base on this location's horizontal position between point1 and point2
// this location's lat,lon is used to calculate the alt of the closest point on the line between point1 and point2
// origin and destination's altitude frames must be the same
// this alt-frame will be updated to match the destination alt frame
void Location::linearly_interpolate_alt(const Location &point1, const Location &point2)
{
    // new target's distance along the original track and then linear interpolate between the original origin and destination altitudes
    set_alt_cm(point1.alt + (point2.alt - point1.alt) * constrain_float(line_path_proportion(point1, point2), 0.0f, 1.0f), point2.get_alt_frame());
}
