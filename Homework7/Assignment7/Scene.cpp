//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Material.hpp"

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
    // TO DO Implement Path Tracing Algorithm here
    Intersection p_inter = intersect(ray);
    if(!p_inter.happened)
    {
        return Vector3f();
    }
    //check p self-luminous
    if(p_inter.m->hasEmission())
    {
        return Vector3f(255,255,255);
    }
    
    //Direct light
    //sampleLight(inter,pdf_light)
    Intersection x_inter;
    float pdf_light= 0.0f;
    sampleLight(x_inter,pdf_light);

    //Get x,ws,NN,emit from inter
    Vector3f p = p_inter.coords;
    Vector3f x = x_inter.coords;
    Vector3f N = p_inter.normal;
    Vector3f NN = x_inter.normal;
    
    //Shoot a ray from p to x
    //maybe ws have problem
    Vector3f x_p = x-p;
    Vector3f ws_dir = x_p.normalized();
    Ray ws(p,ws_dir);
    Intersection ws_inter = intersect(ws);
    
    float pow=x_p.x*x_p.x + x_p.y*x_p.y + x_p.z*x_p.z;

    Vector3f L_dir(0,0,0);
    Vector3f L_indir(0,0,0);

    //if the ray is not blocked in the middle
    if(ws_inter.distance - x_p.norm()  > -EPSILON)
    {
        Vector3f emit=x_inter.emit;
        Vector3f pdf = p_inter.m->eval(ray.direction,ws.direction,N);
        L_dir = emit*pdf*dotProduct(ws.direction,N)*dotProduct(-ws.direction,NN)
                /pow/pdf_light;
    }

    //Test Russian Roulette with probability RussianRoulette
    if(get_random_float() > RussianRoulette)
        return L_dir;

    //wi=sample(wo,N)
    Vector3f wi = p_inter.m->sample(ray.direction,N).normalized();

    //Trace a ray r(p,wi)
    Ray r(p,wi);
    Intersection q = intersect(r);
    
    //If ray r hit a non-emitting object at q
    if(q.happened && !q.m->hasEmission())
    {
        float pdf=p_inter.m->pdf(ray.direction,r.direction,N);
        Vector3f eval = p_inter.m->eval(ray.direction,r.direction,N);
        L_indir = castRay(r,depth+1)
                  *eval *dotProduct(r.direction,N)
                  /pdf/RussianRoulette;
    }

    return L_dir+L_indir;
    
}