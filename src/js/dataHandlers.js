import { COLORS } from "./constants.js";


const isNewNode = (row, prevResult) => {
  // console.log('prevNetworkData:', prevResult.nodes)
  const prevNodes = prevResult.nodes;
  const dupNode = prevNodes.find(node => {
    return node.id === row.content_id || node.id === row.backlink_id
  })
  if(dupNode){
    // console.log(`${row.backlink_text} ${row.content_name} isDup:`, dupNode);
  }
  return !dupNode;
}

const isDupLink = (newLink, connectedId, prevNetwork) => {
  // console.log(newLink.target, newLink.source, connectedId)
  const newLinkSourceId = newLink.source;
  const newLinkTargetId = newLink.target;
  const prevLinks = prevNetwork.links;
  const dupLink = prevLinks.find(link => {
    const dupForward = link.target.id === newLinkTargetId && link.source.id === connectedId;
    const dupBackward = link.source.id === newLinkSourceId && link.target.id === connectedId;
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
    const srcNode = nodes.find(node => node.id === link.source || node.id === link.source.id)
    const tgtNode = nodes.find(node => node.id === link.target || node.id === link.target.id)
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

export const mkNetworkData = (rows, sourceId, prevResult={nodes:[], links:[]}, includeOnlyContents, isForwardlink=false) => {
  const gData = rows.reduce((prevNetwork, row, index) => {
    if(includeOnlyContents){
      if(row.content_id === null){
        return prevNetwork
      }
    }
    // console.log(`${row.backlink_text} is not just backlink. add `)
    const newNodes = isNewNode(row, prevNetwork) ?
    [
      ...prevNetwork.nodes,
      {
        // id: row.content_id || row.backlink_id,
        contentId: row.content_id,
        backlinkId: row.backlink_id,
        text: row.node_text,
        color: row.content_id ? COLORS[row.primary_category] : COLORS.other,
        isContent: row.content_id ? true : false,
        primaryCategory: row.primary_category || 'none',
        backlinkCount: parseInt(row.backlink_count),
        get id(){
          return this.contentId || this.backlinkId
        },
      }
    ]:[
      ...prevNetwork.nodes
    ]

    const newLink = isForwardlink ? {
      target: row.content_id || row.backlink_id,
      source: sourceId
    }:{
      source: row.content_id || row.backlink_id,
      target: sourceId
    }

    const newLinks = isDupLink(newLink, sourceId, prevNetwork) ? [
      ...prevNetwork.links
    ] : [
      ...prevNetwork.links,
      newLink
    ]
    return {
      nodes: newNodes,
      links: newLinks
    }
  }, prevResult)
  console.log('gData before setNeighbors=', gData);
  const gDataWithNeighborsNLinks = _setNeighborsNLinksToEachNode(gData)
  console.log('gData=', gDataWithNeighborsNLinks)
  return gDataWithNeighborsNLinks;
}