import onnx
from onnx_tensorrt.tensorrt_engine import Engine
import pycuda.driver as cuda
import numpy as np
import cv2
import tensorrt as trt
# model = onnx.load("./baseline.onnx")
# onnx.checker.check_model(model)
# # print(onnx.helper.printable_graph(model.graph))

# engine = backend.prepare(model, device='CUDA', max_batch_size=1)

# input_data = cv2.imread('test.jpg', cv2.IMREAD_COLOR).astype(np.float32)
# output_data = engine.run(input_data)[0]
# # print(output_data)
# print(output_data.shape)

# cv2.imshow(output_data)
class HostDeviceMem(object):
    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __str__(self):
        return "Host:\n" + str(self.host) + "\nDevice:\n" + str(self.device)

    def __repr__(self):
        return self.__str__()

def allocate_buffers(engine):
    """
    Allocates all buffers required for the specified engine
    """
    inputs = []
    outputs = []
    bindings = []
    # Iterate over binding names in engine
    for binding in engine:
        # Get binding (tensor/buffer) size
        size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
        # Get binding (tensor/buffer) data type (numpy-equivalent)
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        # Allocate page-locked memory (i.e., pinned memory) buffers
        host_mem = cuda.pagelocked_empty(size, dtype)
        # Allocate linear piece of device memory
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        # Append the device buffer to device bindings
        bindings.append(int(device_mem))
        # Append to inputs/ouputs list
        if engine.binding_is_input(binding):
            inputs.append(HostDeviceMem(host_mem, device_mem))
        else:
            outputs.append(HostDeviceMem(host_mem, device_mem))
    # Create a stream (to eventually copy inputs/outputs and run inference)
    stream = cuda.Stream()
    return inputs, outputs, bindings, stream

def infer(context, bindings, inputs, outputs, stream, batch_size=1):
    """
    Infer outputs on the IExecutionContext for the specified inputs
    """
    # Transfer input data to the GPU
    [cuda.memcpy_htod_async(inp.device, inp.host, stream) for inp in inputs]
    # Run inference
    context.execute_async(bindings=bindings, stream_handle=stream.handle)
    # Transfer predictions back from the GPU
    [cuda.memcpy_dtoh_async(out.host, out.device, stream) for out in outputs]
    # Synchronize the stream
    stream.synchronize()
    # Return the host outputs
    return [out.host for out in outputs]

trt_engine_path = './baseline.trt'
TRT_LOGGER = trt.Logger()

input_data = cv2.imread('test.jpg', cv2.IMREAD_COLOR)
input_data = (input_data / 255.).astype(np.float32)
# output_data = engine.run(input_data)[0]
# # print(output_data)
# print(output_data.shape)



# Read the serialized ICudaEngine
with open(trt_engine_path, 'rb') as f, trt.Runtime(TRT_LOGGER) as runtime:
    # Deserialize ICudaEngine
    engine = runtime.deserialize_cuda_engine(f.read())
# Now just as with the onnx2trt samples...
# Create an IExecutionContext (context for executing inference)
with engine.create_execution_context() as context:
    # Allocate memory for inputs/outputs
    inputs, outputs, bindings, stream = allocate_buffers(engine)
    # Set host input to the image
    # inputs[0].host = image
    inputs[0].host = input_data
    # Inference
    trt_outputs = infer(context, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)
    # Prediction
    # pred_id = np.argmax(trt_outputs[-1])
    output = trt_outputs[0]
    print(output.shape)

outputs = output.reshape((32, 224, 224))
for i, output in enumerate(outputs):
    output = (outputs[0] * 255).astype(np.uint8)
    print(output.max(), output.min(), output.dtype)
    cv2.imwrite(f'outputs{i}.jpg', output)
