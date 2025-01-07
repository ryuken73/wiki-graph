import React from 'react';
import styled from 'styled-components';
// import {ForceGraph2D} from 'react-force-graph';
import Graph2D from './Graph2D'
import {
  getBacklinksByContentId,
  getForwardlinksByBacklinkId
} from './js/serverApi.js';
import {mkNetworkData} from './js/dataHandlers.js';
import {
  getNodeIdsConnected, 
  getLinksOfNode,
  removeNodes
} from './js/graphHandlers.js';
import './App.css'
import NodesShown from './Components/NodesShown';
import ForwardlinkContainer from './Components/ForwardlinkContainer.jsx';
import BacklinkContainer from './Components/BacklinkContainer.jsx';

const Container = styled.div``
const AbsoluteBoxForNodesShown = styled.div`
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  grid-gap: 5px;
  position:  absolute;
  top: 15vh;
  right: 5%;
  z-index: 10;  
  /* background: rgba(255,255,255,0.1); */
  /* height: 80%; */
`

const COLORS = {
  person: 'yellow',
  other: 'grey'
}
const contentId = '가수_한국_C_004301_유재석';
const initialNode = {
  id:  contentId,
  text: '유재석',
  color: COLORS.person,
  value: 100
}

function App() {
  const [lastNetworkData, setLastNetworkData] = React.useState({nodes:[initialNode], links:[]});
  const [nodesExpanded, setNodesExpanded] = React.useState([initialNode]);
  const [activeExpandedNodeId, setActiveExpandedNodeId] = React.useState(null);
  const [backlinksToShow, setBacklinksToShow] = React.useState([]);
  // get initial network data
  React.useEffect( () => {
    getBacklinksByContentId(contentId)
    .then((result: unknown) => {
      console.log(result)
      const includeOnlyContents = true;
      setLastNetworkData((lastNetworkData) => {
        const newNetworkData = mkNetworkData(result, contentId, lastNetworkData, includeOnlyContents);
        return newNetworkData;
      })
    })
  }, [])
  const expandNode = React.useCallback(async (node) => {
    console.log('node click:', node)
    const {id, isContent} = node;
    console.log(isContent)
    if(!isContent){
      return false
    }
    // if(nodesExpanded.some(node => node.id === id)){
    //   return false
    // }
    console.log(node);
    const includeOnlyContents = true;
    const rows = await getBacklinksByContentId(id)
    // const newNetworkData = mkNetworkData(rows, node.id, lastNetworkData, includeOnlyContents);
    // setLastNetworkData(newNetworkData)
    setLastNetworkData(lastNetworkData => {
      return mkNetworkData(rows, node.id, lastNetworkData, includeOnlyContents);
    })
    setNodesExpanded(nodes => {
      const isDup = nodes.some(existingNode => existingNode.id === node.id);
      return isDup ? nodes : [node, ...nodes]
    })
  }, []);
  const expandNodeWithForwardLinks = React.useCallback(async (node) => {
    console.log('node click', node);
    const {backlinkId} = node;
    const includeOnlyContents = true;
    const isForwardlink = true;
    const rows = await getForwardlinksByBacklinkId(backlinkId)
    const newNetworkData = mkNetworkData(rows, node.id, lastNetworkData, includeOnlyContents, isForwardlink);
    setLastNetworkData(newNetworkData)
    // setNodesExpanded(nodes => [node, ...nodes]);
    setNodesExpanded(nodes => {
      const isDup = nodes.some(existingNode => existingNode.id === node.id);
      return isDup ? nodes : [node, ...nodes]
    })
  }, [lastNetworkData])

  const removeNode = React.useCallback((event) => {
    const id = event.target.id
    setLastNetworkData((lastNetworkData) => {
      const includeSelf = true;
      const nodeIdsConnected = getNodeIdsConnected(lastNetworkData, id, includeSelf);
      const nodeIdsExpanded = nodesExpanded.map(node => node.id);
      const nodeIdsToDelete = removeNodes(nodeIdsConnected, nodeIdsExpanded);
      console.log(nodeIdsToDelete)
      const newLinks = [...lastNetworkData.links].filter(link => {
        return !nodeIdsToDelete.includes(link.source.id) && !nodeIdsToDelete.includes(link.target.id);
      })
      const newNodes = [...lastNetworkData.nodes].filter(node => {
        return !nodeIdsToDelete.includes(node.id)
      })
      return {
        nodes: newNodes,
        links: newLinks
      };
    })
    setNodesExpanded(nodesExpanded => {
      return [...nodesExpanded].filter(node => {
        return node.id !== id;
      })
    })
  }, [nodesExpanded])

  console.log('lastNetwokData=', lastNetworkData)

  return (
    <Container>
      <Graph2D
        graphData={lastNetworkData}
        handleNodeClick={expandNode}
        handleLeftClick={expandNodeWithForwardLinks}
      ></Graph2D>
      <AbsoluteBoxForNodesShown>
        <BacklinkContainer
          lastNetworkData={lastNetworkData}
          expandNode={expandNode}
          activeExpandedNodeId={activeExpandedNodeId}
          backlinksToShow={backlinksToShow}
        ></BacklinkContainer>
        <NodesShown
          nodesExpanded={nodesExpanded}
          removeNode={removeNode}
          lastNetworkData={lastNetworkData}
          setLastNetworkData={setLastNetworkData}
          setBacklinksToShow={setBacklinksToShow}
          setActiveExpandedNodeId={setActiveExpandedNodeId}
        ></NodesShown>
        <ForwardlinkContainer
          nodesExpanded={nodesExpanded}
          removeNode={removeNode}
          setLastNetworkData={setLastNetworkData}
        ></ForwardlinkContainer>
      </AbsoluteBoxForNodesShown>
    </Container>
  )
}

export default App
