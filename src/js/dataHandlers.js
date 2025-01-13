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
  console.log(newLink.target, newLink.source, connectedId)
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

export const mkNetworkData = (rows, sourceId, prevResult={nodes:[], links:[]}, includeOnlyContents, isForwardlink=false) => {
  const gData = rows.reduce((acct, row, index) => {
    if(includeOnlyContents){
      if(row.content_id === null){
        return acct
      }
    }
    // console.log(`${row.backlink_text} is not just backlink. add `)
    const newNodes = isNewNode(row, prevResult) ?
    [
      ...acct.nodes,
      {
        // id: row.content_id || row.backlink_id,
        contentId: row.content_id,
        backlinkId: row.backlink_id,
        text: row.backlink_text,
        color: row.content_id ? COLORS[row.primary_category] : COLORS.other,
        isContent: row.content_id ? true : false,
        primaryCategory: row.primary_category || 'none',
        backlinkCount: parseInt(row.backlink_count),
        get id(){
          return this.contentId || this.backlinkId
        },
      }
    ]:[
      ...acct.nodes
    ]

    const newLink = isForwardlink ? {
      target: row.content_id || row.backlink_id,
      source: sourceId
    }:{
      source: row.content_id || row.backlink_id,
      target: sourceId
    }

    const newLinks = isDupLink(newLink, sourceId, prevResult) ? [
      ...acct.links
    ] : [
      ...acct.links,
      newLink
    ]
    return {
      nodes: newNodes,
      links: newLinks
    }
  }, prevResult)
  console.log('gData=', gData)
  // if(gData.links.length === 0){
  //   return gData;
  // }
  // gData.links.forEach(link => {
  //   console.log(link)
  //   // const a = gData.nodes[link.source];
  //   // const b = gData.nodes[link.target];
  //   const a = gData.nodes.find(node => node.id === link.source);
  //   const b = gData.nodes.find(node => node.id === link.target);
  //   console.log(a, b)
  //   !a.neighbors && (a.neighbors = []);
  //   !b.neighbors && (b.neighbors = []);
  //   a.neighbors.push(b);
  //   b.neighbors.push(a);

  //   !a.links && (a.links = []);
  //   !b.links && (b.links = []);
  //   a.links.push(link);
  //   b.links.push(link);
  // });
  return gData;
}