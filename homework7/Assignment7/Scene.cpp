//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TODO Implement Path Tracing Algorithm here

    Intersection p = Scene::intersect(ray);
    if(!p.happened) 
    {
      return Vector3f(0.f);
    }

    return Scene::shade(p, -ray.direction);
}

Vector3f Scene::shade(Intersection &p, const Vector3f &wo) const
{

    if(p.m->hasEmission())
    { 
      return p.m->getEmission();
    }

    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light, pdf_light);

    Vector3f light_dir;

    double p2light_dist = (inter_light.coords - p.coords).norm();
    Vector3f p2light = (inter_light.coords - p.coords).normalized();
    Intersection occluding = Scene::intersect(Ray(p.coords, p2light));   
    if(occluding.distance < 0) std::cout<< "1 intersection distance < 0" << std::endl;
    if(!occluding.happened || p2light_dist - occluding.distance < epsilon)
    {
      light_dir = inter_light.emit * p.m->eval(wo, p2light, p.normal) * std::max<float>(0.f, dotProduct(p2light, p.normal)) * std::max<float>(0.f, dotProduct(-p2light, inter_light.normal)) / (p2light_dist * p2light_dist) / pdf_light;
    }

    
    Vector3f light_indir;

    float P_RR = get_random_float();
    if(P_RR > RussianRoulette)
      return light_dir;
    
    Vector3f wi = p.m->sample(wo, p.normal);
    Intersection q = Scene::intersect(Ray(p.coords, wi));
    if(q.distance < 0) std::cout<< "2 intersection distance < 0" << std::endl;
    if(q.happened && !(q.obj->hasEmit()))
    {
      light_indir = shade(q, -wi) * p.m->eval(wo, wi, p.normal) * std::max<float>(0.f, dotProduct(wi, p.normal)) / p.m->pdf(wo, wi, p.normal) / RussianRoulette; 
    }
    
    return light_dir + light_indir;
      

}
