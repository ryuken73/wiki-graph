import { COLORS } from "./constants.js";


const isNewNode = (row, prevNodes) => {
  // console.log('prevNetworkData:', prevResult.nodes)
  const dupNode = prevNodes.find(node => {
    return node.id === row.content_id || node.id === row.backlink_id
  })
  if(dupNode){
    // console.log(`${row.backlink_text} ${row.content_name} isDup:`, dupNode);
  }
  return !dupNode;
}

const isDupLink = (newLink, connectedId, prevLinks) => {
  // console.log(newLink.target, newLink.source, connectedId)
  const newLinkSourceId = newLink.source;
  const newLinkTargetId = newLink.target;
  const dupLink = prevLinks.find(link => {
    const dupForward = link.target?.id === newLinkTargetId && link.source?.id === connectedId;
    const dupBackward = link.source?.id === newLinkSourceId && link.target?.id === connectedId;
    // console.log(dupForward, dupBackward)
    return dupForward || dupBackward
  })
  // console.log('isDupLink value is', dupLink)
  return dupLink;
}

export const extractBacklinks = (networkData, centerId) => {
  console.log(networkData, centerId)
  const {links} = networkData;
  const linksRelated = links.filter(link => {
    return link.target.id === centerId;
  })
  return [...new Set(linksRelated.map(link => link.source))];
}

export const extractForwardlinks = (networkData, centerId) => {
  const {links} = networkData;
  const linksRelated = links.filter(link => {
    return link.source.id === centerId;
  })
  return [...new Set(linksRelated.map(link => link.target))];
}
const _setNeighborsNLinksToEachNode = (networkData) => {
  const links = [...networkData.links];
  const nodes = [...networkData.nodes];
  links.forEach(link => {
    const srcNode = nodes.find(node => node.id === link.source || node.id === link.source?.id)
    const tgtNode = nodes.find(node => node.id === link.target || node.id === link.target?.id)
    // add neighbors to each node
    if(srcNode){
      !srcNode.neighbors && (srcNode.neighbors = []);
      srcNode.neighbors.push(tgtNode);
    }
    if(tgtNode){
      !tgtNode.neighbors && (tgtNode.neighbors = []);
      tgtNode.neighbors.push(srcNode);
    }
    // add links to each node
    srcNode && !srcNode.links && (srcNode.links = []);
    tgtNode && !tgtNode.links && (tgtNode.links = []);
    srcNode && srcNode.links.push(link);
    tgtNode && tgtNode.links.push(link);
  })
  return {nodes, links} 
}

const _mkNewNodes = (row, prevNodes) => {
  const newNode = _mkNewNode(row);
  return isNewNode(row, prevNodes) ?
  [
    ...prevNodes,
    newNode
  ]:[
    ...prevNodes
  ]
}
const _mkNewNode = (row) => {
  return ({
    contentId: row.content_id,
    backlinkId: row.backlink_id,
    text: row.node_text,
    color: row.content_id ? COLORS[row.primary_category] : COLORS.other,
    isContent: row.content_id ? true : false,
    primaryCategory: row.primary_category || 'none',
    backlinkCount: parseInt(row.backlink_count) || 0,
    backlinkCountContent: parseInt(row.backlink_count_from_content) || 0,
    forwardlinkCount: parseInt(row.forwardlink_count) || 0,
    forwardlinkCountContent: parseInt(row.forwardlink_count_to_content) || 0,
    additionalInfo: row.additional_info_raw,
    imageName: row.image_name,
    imageSubdir: row.image_subdir,
    wikiUrl: row.node_url,
    get id(){
      return this.contentId || this.backlinkId
    },
  })
}
const _mkNewLink = (row, centerNodeId, isForwardlink) => {
  return (
    isForwardlink ? {
      target: row.content_id || row.backlink_id,
      source: centerNodeId
    }:{
      source: row.content_id || row.backlink_id,
      target: centerNodeId
    }
  )
}

const _mkNewLinks = (centerNodeId, row, prevLinks, isForwardlink) => {
  const newLink = _mkNewLink(row, centerNodeId, isForwardlink);
  return isDupLink(newLink, centerNodeId, prevLinks) ? [
    ...prevLinks
  ] : [
    ...prevLinks,
    newLink
  ]
}
const _addNodesToNetworkData = (row, prevNetwork) => {
  const newNodes = _mkNewNodes(row, prevNetwork.nodes)
  return {
    nodes: newNodes,
    links: prevNetwork.links
  }
}
export const addNewNodeNExpandNetworkData = (newNode, expandNodes, prevNetwork, includeOnlyContents, isForwradlink=false) => {
  const newNodeAddedNetwork = _addNodesToNetworkData(newNode, prevNetwork);
  console.log('newNodeAdded:', newNodeAddedNetwork);
  const newNodeId = newNode.content_id || newNode.backlink_id;
  return expandNetworkData(expandNodes, newNodeId, newNodeAddedNetwork, includeOnlyContents, isForwradlink);
}

export const expandNetworkData = (rows, centerNodeId, prevResult={nodes:[], links:[]}, includeOnlyContents, isForwardlink=false) => {
  const gData = rows.reduce((prevNetwork, row) => {
    if(includeOnlyContents){
      if(row.content_id === null){
        return prevNetwork
      }
    }
    const newNodes = _mkNewNodes(row, prevNetwork.nodes)
    const newLinks = _mkNewLinks(centerNodeId, row, prevNetwork.links, isForwardlink);

    return {
      nodes: newNodes,
      links: newLinks
    }
  }, prevResult)
  console.log('gData before setNeighbors=', gData);
  const gDataWithNeighborsNLinks = _setNeighborsNLinksToEachNode(gData)
  console.log('gData=', gDataWithNeighborsNLinks)
  return gDataWithNeighborsNLinks;
  // return gData;
}