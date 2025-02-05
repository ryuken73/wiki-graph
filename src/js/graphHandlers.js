
export const getNodeIdsConnected = (lastNetworkData, nodeId) => {
  const {links} = lastNetworkData;
  const neighbors = links.reduce((acct, link) => {
    // console.log(acct)
    if(link.source.id === nodeId){
      return [...acct, link.target.id]
    }
    if(link.target.id === nodeId){
      return [...acct, link.source.id]
    }
    return [...acct]
  }, [])
  return neighbors;
}

export const getLinksOfNode = (lastNetworkData, nodeId) => {
  const newLinks = [...lastNetworkData.nodes]
  return newLinks.filter(link => {
    return link.source.id === nodeId || link.target.id === nodeId;
  })
}

export const isLastLeafNode = (lastNetwork, centerNodeId, nodeId) => {
  const {links} = lastNetwork;
  const hasOtherConnectedNode = links.some(link => {
    let isLinkOfOtherNode = false;
    const isSourceNode = link.source.id === nodeId;
    const isTargetNode = link.target.id === nodeId;
    if(isSourceNode){
      isLinkOfOtherNode = link.target.id !== centerNodeId;
    }
    if(isTargetNode){
      isLinkOfOtherNode = link.source.id !== centerNodeId;
    }
    return isLinkOfOtherNode;
  })
  return !hasOtherConnectedNode
}

export const notInNodes = (nodes, nodeId) => {
  console.log(nodes, nodeId)
  return nodes.every(node => node.id !== nodeId);
}

export const removeNodes = (nodesFrom, nodesToRemove) => {
  const newNodes = [...nodesFrom];
  return newNodes.reduce((acct, newNode) => {
    if(nodesToRemove.includes(newNode)){
      return acct
    }
    return [...acct, newNode]
  }, [])
}

export const getNodeTextById = (lastNetworkData, nodeId) => {
  return (lastNetworkData.nodes.find(node => node.id === nodeId))?.text;
}
export const getNumberOfNodes = (lastNetworkData) => {
  return lastNetworkData.nodes.length;
}

export const genFocusNode = (graphRef, dimension) => (node) => {
  const distance = 40;
  const distRatio = 1 + distance/Math.hypot(node.x, node.y, node.z);

  const TRANSITION_MS = 1000;
  console.log(graphRef)
  if(dimension === '2D'){
    console.log('center to', node.x, node.y)
    graphRef.current.centerAt( node.x, node.y, TRANSITION_MS)
    // graphRef.current.centerAt();
  }
}
  
