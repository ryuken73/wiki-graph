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
  removeNodes,
  getNodeTextById,
  getNumberOfNodes,
  genFocusNode
} from './js/graphHandlers.js';
import './App.css'
import ExpandedContainer from './Components/ExpandedContainer';
import ForwardlinkContainer from './Components/ForwardlinkContainer.jsx';
import BacklinkContainer from './Components/BacklinkContainer.jsx';
import AutoComplete from './Components/AutoComplete.jsx';

const Container = styled.div``
const AbsoluteBoxForSearch = styled.div`
  position: absolute;
  top: 5vh;
  left: 5%;
`
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
  padding: 5px;
  border: 2px solid rgba(255, 255, 255, 0.1);
`
const ActiveExpandedNode = styled.div`
  grid-column: 1/-1;
  color: yellow;
  font-weight: 200;
  background: rgba(255,255,255,0.1);
  padding: 3px; 
  font-size: 20px;
`
const NodeText = styled.span``
const TotalNodes = styled.span`
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
  const [forwardlinksToShow, setForwardlinkToShow] = React.useState([]);
  const graphRef = React.useRef(null);

  const focusNode2D = genFocusNode(graphRef, '2D');

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
    setActiveExpandedNodeId(node.id);
    // focusNode2D(node)
  }, []);
  const expandNodeWithForwardLinks = React.useCallback(async (node) => {
    console.log('node click', node);
    const {backlinkId} = node;
    const includeOnlyContents = true;
    const isForwardlink = true;
    const rows = await getForwardlinksByBacklinkId(backlinkId)
    setLastNetworkData(lastNetworkData => {
      return mkNetworkData(rows, node.id, lastNetworkData, includeOnlyContents, isForwardlink);
    })
    setNodesExpanded(nodes => {
      const isDup = nodes.some(existingNode => existingNode.id === node.id);
      return isDup ? nodes : [node, ...nodes]
    })
    setActiveExpandedNodeId(node.id);
  }, [])

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
  const totalNodes = getNumberOfNodes(lastNetworkData);
  const activeExpandedText = getNodeTextById(lastNetworkData, activeExpandedNodeId);

  return (
    <Container>
      <Graph2D
        ref={graphRef}
        graphData={lastNetworkData}
        handleNodeClick={expandNode}
        handleLeftClick={expandNodeWithForwardLinks}
      ></Graph2D>
      <AbsoluteBoxForSearch>
        <AutoComplete></AutoComplete>
      </AbsoluteBoxForSearch>
      <AbsoluteBoxForNodesShown>
        <ActiveExpandedNode>
          {activeExpandedNodeId !== null ? (
            <NodeText>{activeExpandedText}</NodeText>
          ) : (
            <TotalNodes 
            >[{totalNodes}]</TotalNodes>
          )}
        </ActiveExpandedNode>
        <BacklinkContainer
          lastNetworkData={lastNetworkData}
          expandNode={expandNode}
          activeExpandedNodeId={activeExpandedNodeId}
          backlinksToShow={backlinksToShow}
        ></BacklinkContainer>
        <ExpandedContainer
          nodesExpanded={nodesExpanded}
          removeNode={removeNode}
          lastNetworkData={lastNetworkData}
          setLastNetworkData={setLastNetworkData}
          setBacklinksToShow={setBacklinksToShow}
          setForwardlinksToShow={setForwardlinkToShow}
          setActiveExpandedNodeId={setActiveExpandedNodeId}
          focusNode={focusNode2D}
        ></ExpandedContainer>
        <ForwardlinkContainer
          lastNetworkData={lastNetworkData}
          expandNode={expandNode}
          activeExpandedNodeId={activeExpandedNodeId}
          forwardlinksToShow={forwardlinksToShow}
        ></ForwardlinkContainer>
      </AbsoluteBoxForNodesShown>
    </Container>
  )
}

export default App
