export const getNodesConnected = (lastNetworkData, nodeId) => {
  const {links} = lastNetworkData;
  return links.reduce((acct, link) => {
    console.log(acct)
    if(link.source.id === nodeId){
      return [...acct, link.target.id]
    }
    if(link.target.id === nodeId){
      return [...acct, link.source.id]
    }
    return [...acct]
  }, [])
}

export const getLinksOfNode = (lastNetworkData, nodeId) => {
  const newLinks = [...lastNetworkData.nodes]
  return newLinks.filter(link => {
    return link.source.id === nodeId || link.target.id === nodeId;
  })
}

export const removeNodes = (lastNetworkData, nodesToRemove) => {
  const newNodes = [...lastNetworkData.nodes];
  return newNodes.filter(node => {
    if(nodesToRemove.find(nodeToRemove => nodeToRemove.id === node.id)){
      return false;
    }
    return true;
  })
}
  
