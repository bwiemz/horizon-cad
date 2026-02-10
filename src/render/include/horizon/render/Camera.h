#pragma once

#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

#include <utility>

namespace hz::render {

enum class ProjectionType { Perspective, Orthographic };

class Camera {
public:
    Camera();

    void setPerspective(double fovY, double aspect, double nearPlane, double farPlane);
    void setOrthographic(double width, double height, double nearPlane, double farPlane);

    void lookAt(const math::Vec3& eye, const math::Vec3& target, const math::Vec3& up);

    void orbit(double deltaYaw, double deltaPitch);
    void pan(double deltaX, double deltaY);
    void zoom(double factor);

    void fitAll(const math::BoundingBox& bbox);

    void setFrontView();
    void setTopView();
    void setRightView();
    void setIsometricView();

    math::Mat4 viewMatrix() const;
    math::Mat4 projectionMatrix() const;
    math::Mat4 viewProjectionMatrix() const;

    std::pair<math::Vec3, math::Vec3> screenToRay(double screenX, double screenY,
                                                   int vpW, int vpH) const;
    math::Vec3 unproject(double screenX, double screenY, double depth,
                         int vpW, int vpH) const;

    const math::Vec3& eye() const { return m_eye; }
    const math::Vec3& target() const { return m_target; }
    ProjectionType projectionType() const { return m_projType; }

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
};

}  // namespace hz::render
