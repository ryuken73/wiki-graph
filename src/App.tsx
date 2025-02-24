import React from 'react';
import styled from 'styled-components';
// import {ForceGraph2D} from 'react-force-graph';
import Graph2D from './Graph2D';
import Backdrop from './Components/Backdrop';
import {
  getBacklinksByContentId,
  getForwardlinksByBacklinkId,
  getNodeByContentId,
  getNodeByBacklinkId,
  getRelatedLinks
} from './js/serverApi.js';
import {
  expandNetworkData,
  addNewNodeNExpandNetworkData
} from './js/dataHandlers.js';
import {
  isLastLeafNode,
  notInNodes,
  getNodeIdsConnected, 
  getLinksOfNode,
  getOnlyNeighbors,
  isOnlyLink,
  isLinkOfNode,
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
import QuickButtons from './Components/QuickButtons.jsx';
// import NodeHandler from './Components/NodeHandler';

const Container = styled.div``
const AbsoluteBoxForSearch = styled.div`
  position: absolute;
  top: 5vh;
  left: 5%;
`
const AbsoluteBoxForNodesShown = styled.div`
  display: grid;
  /* grid-template-columns: 1fr 1fr 1fr; */
  grid-template-columns: 1fr;
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

const openChildWindow = (wikiUrl, windowFeatures) => {
  console.log('open', wikiUrl)
  window.open(`https://namu.wiki${wikiUrl}`, "aa", `width=800,height=1000,${windowFeatures}`);
}

function App() {
  // const [lastNetworkData, setLastNetworkData] = React.useState({nodes:[initialNode], links:[]});
  // const [nodesExpanded, setNodesExpanded] = React.useState([initialNode]);
  const [lastNetworkData, setLastNetworkData] = React.useState({nodes:[], links:[]});
  const [nodesExpanded, setNodesExpanded] = React.useState([]);
  const [lastNodeExpanded, setLastNodeExpanded] = React.useState(null);
  const [activeExpandedNodeId, setActiveExpandedNodeId] = React.useState(null);
  const [backlinksToShow, setBacklinksToShow] = React.useState([]);
  const [forwardlinksToShow, setForwardlinkToShow] = React.useState([]);
  const [showBackdrop, setShowBackdrop] = React.useState(false);
  const [checkedNodeList, setCheckedNodeList] = React.useState([]);
  const graphRef = React.useRef(null);
  const focusNode2D = genFocusNode(graphRef, '2D');

  console.log(checkedNodeList)

  const addCheckedNodeList = React.useCallback((nodeId) => {
    const targetNode = nodesExpanded.find(node => node.id === nodeId);
    setCheckedNodeList((checkedNodeList) => {
      return [
        ...checkedNodeList,
        targetNode
      ];
    })
  }, [nodesExpanded]);
  const delCheckedNodeList = React.useCallback((nodeId) => {
    setCheckedNodeList((checkedNodeList) => {
      const newCheckedNodeList = [...checkedNodeList];
      return newCheckedNodeList.filter(node => node.id !== nodeId)
    })
  }, []);
  const postExpandTask = React.useCallback((node) => {
    console.log('postExpandTask node =', node)
    setNodesExpanded((nodes) => {
      const isDup = nodes.some(existingNode => existingNode.id === node.id);
      return isDup ? nodes : [node, ...nodes]
    })
    setActiveExpandedNodeId(node.id);
    setLastNodeExpanded(node);
    // focusNode2D(node)
  }, [])

  const addNewNode = React.useCallback(async (nodeId, isNodeContent) => {
    console.log('addNewNode nodeId =', nodeId)
    setShowBackdrop(true)
    const nodeInfo = isNodeContent ? await getNodeByContentId(nodeId) : await getNodeByBacklinkId(nodeId);
    const expandNodes = isNodeContent ? await getBacklinksByContentId(nodeId) : await getForwardlinksByBacklinkId(nodeId)
    const newNode = nodeInfo[0];
    const includeOnlyContents = true;
    setLastNetworkData((lastNetworkData) => {
      const isForwarding = !isNodeContent;
      const newNetworkData = addNewNodeNExpandNetworkData(newNode, expandNodes, lastNetworkData, includeOnlyContents, isForwarding);
      // const addedNode = newNetworkData.nodes.find(node => node.id === nodeId);
      const addedNode = newNetworkData.nodes.find(node => {
        // 새로운 content가 검색엔진에 반영이 되지 않는 경우,
        // nodeId는 backlink_id인데 networkData에는 content_id로 id가 mapping되어 오류가 발생할 수 있음.
        return node.id === nodeId || node.backlinkId === nodeId;
      })
      setTimeout(() => {
        postExpandTask(addedNode)
      }, 150);
      setShowBackdrop(false)
      return newNetworkData;
    })
  }, [postExpandTask]);

  // get initial network data
  React.useEffect( () => {
    const IS_NODE_CONTENT = true;
    // addNewNode(contentId, IS_NODE_CONTENT);
  }, [addNewNode])

  const expandNode = React.useCallback(async (node, isForwardlink=false) => {
    setShowBackdrop(true)
    const {id, backlinkId, isContent} = node;
    // console.log(isContent)
    // if(!isContent){
    //   setShowBackdrop(false)
    //   return false
    // }
    console.log(node);
    const includeOnlyContents = true;
    const rows = isForwardlink ?
      await getForwardlinksByBacklinkId(backlinkId) :
      await getBacklinksByContentId(id)
    setLastNetworkData(lastNetworkData => {
      setTimeout(() => {
        postExpandTask(node);
      }, 150)
      return expandNetworkData(rows, node.id, lastNetworkData, includeOnlyContents, isForwardlink);
    })
    setShowBackdrop(false)
  }, [postExpandTask]);

  const expandRelatedNode = React.useCallback(async (nodes, isForwardlink=false) => {
    console.log('nodes:', nodes)
    setShowBackdrop(true)
    const direction = isForwardlink ? 'forwardlink':'backlink'
    const rows = await getRelatedLinks(nodes, direction);
    const includeOnlyContents = false;
      setLastNetworkData(lastNetworkData => {
        return expandNetworkData(rows, nodes[0].id, lastNetworkData, includeOnlyContents, isForwardlink);
      })
      setLastNetworkData(lastNetworkData => {
        return expandNetworkData(rows, nodes[1].id, lastNetworkData, includeOnlyContents, isForwardlink);
      })
    // for(let i=0;i++;i<nodes.length){
    //   setLastNetworkData(lastNetworkData => {
    //     return expandNetworkData(rows, nodes[0].id, lastNetworkData, includeOnlyContents, isForwardlink);
    //   })
    // }
    setShowBackdrop(false)
  }, [])

  const shrinkNode = React.useCallback((eventOrId, direction = 'all') => {
    const fromEvent = eventOrId && eventOrId.nativeEvent; 
    const centerNodeId = fromEvent ? eventOrId.target.id : eventOrId ;
    setLastNetworkData((lastNetworkData) => {
      const onlyNeighborsId = getOnlyNeighbors(lastNetworkData, centerNodeId);
      const newNodes = [...lastNetworkData.nodes].filter(node => {
        return !onlyNeighborsId.includes(node.id)
      })
      const newLinks = [...lastNetworkData.links].filter(link => {
        return !isOnlyLink(link, centerNodeId, onlyNeighborsId)
      })
      return {
        nodes: newNodes,
        links: newLinks
      };
    })
    setNodesExpanded(nodesExpanded => {
      return [...nodesExpanded].filter(node => {
        return node.id !== centerNodeId;
      })
    })
  }, [])

  const removeNode = React.useCallback((eventOrId, hiddenParams) => {
    const fromEvent = eventOrId && eventOrId.nativeEvent; 
    const centerNodeId = fromEvent ? eventOrId.target.id : eventOrId ;
    setLastNetworkData((lastNetworkData) => {
      const onlyNeighborsId = getOnlyNeighbors(lastNetworkData, centerNodeId);
      console.log('onlyNeighborsId:', onlyNeighborsId)
      const newNodes = [...lastNetworkData.nodes].filter(node => {
        const onlyNeighborNode = onlyNeighborsId.includes(node.id);
        const isCenterNode = node.id === centerNodeId;
        return !onlyNeighborNode && !isCenterNode;
      })
      console.log('newNodes:', newNodes)
      // const newLinks = getLinksOfNode(lastNetworkData, centerNodeId);
      const newLinks = [...lastNetworkData.links].filter(link => {
        const isConnected = isLinkOfNode(link, centerNodeId);
        return !isConnected;
      })
      return {
        nodes: newNodes,
        links: newLinks
      };
    })
    setNodesExpanded(nodesExpanded => {
      return [...nodesExpanded].filter(node => {
        return node.id !== centerNodeId;
      })
    })
  }, [])
  const removeAllNodes = React.useCallback(() => {
    setLastNetworkData({nodes:[], links:[]});
    setNodesExpanded([]);
  }, [])

  console.log('lastNetwokData=', lastNetworkData)
  const totalNodes = getNumberOfNodes(lastNetworkData);
  const activeExpandedText = getNodeTextById(lastNetworkData, activeExpandedNodeId);

  const onClickNodeText = React.useCallback(() => {
    const node = lastNetworkData.nodes.find(node => node.id === activeExpandedNodeId);
    const wikiUrl = node.wikiUrl;
    openChildWindow(wikiUrl)
  }, [activeExpandedNodeId, lastNetworkData.nodes])

  return (
    <Container>
      <Backdrop
        open={showBackdrop}
        setOpen={setShowBackdrop}
      ></Backdrop>
      {/* <NodeHandler
        checkedNodeList={checkedNodeList}
        setCheckedNodeList={setCheckedNodeList}
        removeNode={removeNode}
        expandNode={expandNode}
        shrinkNode={shrinkNode}
      ></NodeHandler> */}
      <Graph2D
        ref={graphRef}
        graphData={lastNetworkData}
        expandNode={expandNode}
        removeNode={removeNode}
        expandRelatedNode={expandRelatedNode}
        setLastNodeExpanded={setLastNodeExpanded}
      ></Graph2D>
      <AbsoluteBoxForSearch>
        <AutoComplete
          onSuggestSelected={addNewNode}
        ></AutoComplete>
      </AbsoluteBoxForSearch>
      <QuickButtons
        removeAllNodes={removeAllNodes}
      ></QuickButtons>
      <AbsoluteBoxForNodesShown>
        <ActiveExpandedNode>
          {activeExpandedNodeId !== null ? (
            <NodeText
              onClick={onClickNodeText}
            >{activeExpandedText}</NodeText>
          ) : (
            <TotalNodes 
            >[{totalNodes}]</TotalNodes>
          )}
        </ActiveExpandedNode>
        {/* <BacklinkContainer
          lastNetworkData={lastNetworkData}
          expandNode={expandNode}
          activeExpandedNodeId={activeExpandedNodeId}
          backlinksToShow={backlinksToShow}
        ></BacklinkContainer> */}
        <ExpandedContainer
          checkedNodeList={checkedNodeList}
          setCheckedNodeList={setCheckedNodeList}
          nodesExpanded={nodesExpanded}
          removeNode={removeNode}
          expandNode={expandNode}
          shrinkNode={shrinkNode}
          lastNetworkData={lastNetworkData}
          setLastNetworkData={setLastNetworkData}
          setBacklinksToShow={setBacklinksToShow}
          setForwardlinksToShow={setForwardlinkToShow}
          setActiveExpandedNodeId={setActiveExpandedNodeId}
          addCheckedNodeList={addCheckedNodeList}
          delCheckedNodeList={delCheckedNodeList}
          focusNode={focusNode2D}
        
        ></ExpandedContainer>
        {/* <ForwardlinkContainer
          lastNetworkData={lastNetworkData}
          expandNode={expandNode}
          activeExpandedNodeId={activeExpandedNodeId}
          forwardlinksToShow={forwardlinksToShow}
        ></ForwardlinkContainer> */}
      </AbsoluteBoxForNodesShown>
    </Container>
  )
}

export default App
