
export const getNodeIdsConnected = (lastNetworkData, nodeId, includeSelf) => {
  const {links} = lastNetworkData;
  const neighbors = links.reduce((acct, link) => {
    console.log(acct)
    if(link.source.id === nodeId){
      return [...acct, link.target.id]
    }
    if(link.target.id === nodeId){
      return [...acct, link.source.id]
    }
    return [...acct]
  }, [])
  return includeSelf ? [nodeId, ...neighbors]:neighbors;
}

export const getLinksOfNode = (lastNetworkData, nodeId) => {
  const newLinks = [...lastNetworkData.nodes]
  return newLinks.filter(link => {
    return link.source.id === nodeId || link.target.id === nodeId;
  })
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
  
