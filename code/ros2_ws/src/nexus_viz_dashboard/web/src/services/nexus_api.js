/** ROS2/rosbridge 适配层；页面不计算坐标、年龄或实验指标。 */
export function installNexusAPI(store) {
  const api = {
    updatePose: (x, y, z, metadata) => store.updatePose(x, y, z, metadata),
    updateCamera: (data) => store.updateCamera(data),
    updateObservations: (data) => store.updateObservations(data),
    updateMetrics: (data) => store.updateMetrics(data),
    updateStatus: (data) => store.updateStatus(data),
    updateHealth: (data) => store.updateHealth(data),
    addLatency: (value) => store.addLatency(value),
    reset: () => store.reset(),
  };
  window.NexusAPI = Object.freeze(api);
  return api;
}
