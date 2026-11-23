#pragma once

#include <utility>

#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

namespace hz::render {

enum class ProjectionType { Perspective, Orthographic };

class Camera {
public:
    /// The nearest and farthest the camera comes to what it looks at, and the
    /// smallest and largest an orthographic view is: a view held inside them
    /// projects to finite numbers, whatever is zoomed or fitted.
    static constexpr double kMinDistance = 1e-6;
    static constexpr double kMaxDistance = 1e12;

    Camera();

    void setPerspective(double fovY, double aspect, double nearPlane, double farPlane);
    void setOrthographic(double width, double height, double nearPlane, double farPlane);

    /// Clip planes the camera places itself, in proportion to how far it is
    /// from its target, instead of the ones set: a view zoomed far out, or
    /// fitted to a part much larger or smaller than the planes set, still
    /// shows it. Off unless turned on.
    void setAutoClipPlanes(bool on) { m_autoClip = on; }
    bool autoClipPlanes() const { return m_autoClip; }

    /// The clip planes the projection uses: those set, or, with automatic
    /// planes, those placed for the current view.
    double nearPlane() const;
    double farPlane() const;

    void lookAt(const math::Vec3& eye, const math::Vec3& target, const math::Vec3& up);

    void orbit(double deltaYaw, double deltaPitch);
    void pan(double deltaX, double deltaY);
    void zoom(double factor);

    void fitAll(const math::BoundingBox& bbox);

    void setFrontView();
    void setTopView();
    void setRightView();
    void setIsometricView();
    void setBackView();
    void setBottomView();
    void setLeftView();

    math::Mat4 viewMatrix() const;
    math::Mat4 projectionMatrix() const;
    math::Mat4 viewProjectionMatrix() const;

    std::pair<math::Vec3, math::Vec3> screenToRay(double screenX, double screenY, int vpW,
                                                  int vpH) const;
    math::Vec3 unproject(double screenX, double screenY, double depth, int vpW, int vpH) const;

    const math::Vec3& eye() const { return m_eye; }
    const math::Vec3& target() const { return m_target; }
    const math::Vec3& up() const { return m_up; }
    ProjectionType projectionType() const { return m_projType; }
    double fieldOfView() const { return m_fov; }
    double orthoWidth() const { return m_orthoWidth; }
    double orthoHeight() const { return m_orthoHeight; }

private:
    math::Vec3 m_eye{5.0, 5.0, 5.0};
    math::Vec3 m_target{0.0, 0.0, 0.0};
    math::Vec3 m_up{0.0, 0.0, 1.0};

    ProjectionType m_projType = ProjectionType::Perspective;

    double m_fov = 45.0;
    double m_aspect = 1.33;
    double m_near = 0.1;
    double m_far = 10000.0;

    double m_orthoWidth = 100.0;
    double m_orthoHeight = 75.0;

    bool m_autoClip = false;
};

}  // namespace hz::render
