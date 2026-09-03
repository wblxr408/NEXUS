/** ROS2/rosbridge 适配层；页面不计算坐标、年龄或实验指标。 */
export function installNexusAPI(store) {
  const api = {
    updatePose: (x, y, z, metadata) => store.updatePose(x, y, z, metadata),
    updateSolverProvenance: (data) => store.updateSolverProvenance(data),
    updateCamera: (data) => store.updateCamera(data),
    updateEgoPose: (data) => store.updateEgoPose(data),
    updateObservations: (data) => store.updateObservations(data),
    updateMetrics: (data) => store.updateMetrics(data),
    updateStatus: (data) => store.updateStatus(data),
    updateHealth: (data) => store.updateHealth(data),
    updateCarlaStatus: (data) => store.updateCarlaStatus(data),
    updateAlgorithms: (data) => store.updateAlgorithms(data),
    appendLog: (entry) => store.appendLog(entry),
    addLatency: (value) => store.addLatency(value),
    reset: () => store.reset(),
  };
  window.NexusAPI = Object.freeze(api);
  return api;
}
