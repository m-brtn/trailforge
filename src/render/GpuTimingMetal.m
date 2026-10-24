// GPU frame time on Metal: MTLCommandBuffer GPUStartTime/GPUEndTime of a completed SDL GPU command buffer.
// SDL GPU has no timestamp queries, so this reads the command buffer from SDL's Metal fence.
// The struct below mirrors the first member of MetalFence in SDL 3.4.18 (src/gpu/metal/SDL_gpu_metal.m);
// SDL keeps fence->commandBuffer alive until the last fence reference is released.

#import <Metal/Metal.h>

typedef struct TfSdlMetalFenceView {
    id<MTLCommandBuffer> commandBuffer;
} TfSdlMetalFenceView;

double tf_metal_fence_gpu_ms(const void* fence) {
    if (!fence) return -1.0;
    id<MTLCommandBuffer> cb = ((const TfSdlMetalFenceView*)fence)->commandBuffer;
    if (cb == nil || cb.status != MTLCommandBufferStatusCompleted) return -1.0;
    const CFTimeInterval t0 = cb.GPUStartTime, t1 = cb.GPUEndTime;
    return t1 > t0 ? (t1 - t0) * 1000.0 : -1.0;
}
